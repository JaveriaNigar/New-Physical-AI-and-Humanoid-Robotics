"""
Chat API endpoints for the RAG Chatbot backend.
Handles chat queries and selected-text queries.
"""
from fastapi import APIRouter, HTTPException, status
from typing import Union, Dict
import logging
import time

from src.models.chat import ChatQueryRequest, SelectedTextQueryRequest, ChatQueryResponse, RefusalResponse
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.services.validation_service import validation_service
from src.utils.monitoring import monitoring_service
from src.config.settings import settings
from src.utils.logging_config import (
    log_api_request, log_retrieval_event, log_generation_event, log_error
)


router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/query", response_model=Union[ChatQueryResponse, RefusalResponse])
async def chat_query(request: ChatQueryRequest):
    """
    Handle a chat query about textbook content.

    Args:
        request: Chat query request with question and session information

    Returns:
        ChatQueryResponse with the answer or RefusalResponse if no context found
    """
    start_time = time.time()
    monitoring_service.metrics_collector.start_request()

    try:
        # Validate the incoming request
        if not validation_service.validate_question(request.question):
            monitoring_service.record_request(
                response_time=time.time() - start_time,
                has_error=True
            )
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid question provided"
            )

        # Log the query for analytics and debugging
        logger.info(f"Processing chat query for session {request.session_id}: {request.question[:100]}...")

        # Retrieve relevant chunks from the vector database
        retrieval_start = time.time()
        relevant_chunks = retrieval_service.retrieve_relevant_chunks(
            query=request.question,
            limit=10,  # Retrieve up to 10 relevant chunks
            min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD  # Use configured threshold
        )
        retrieval_time = time.time() - retrieval_start

        # Log retrieval event
        log_retrieval_event(
            query=request.question,
            session_id=request.session_id,
            num_chunks_found=len(relevant_chunks),
            retrieval_time=retrieval_time,
            similarity_threshold=0.5
        )

        # If no relevant chunks found, return a refusal response
        if not relevant_chunks:
            logger.info(f"No relevant context found for question in session {request.session_id}")

            response_time = time.time() - start_time
            monitoring_service.record_request(
                response_time=response_time,
                retrieval_time=retrieval_time
            )

            return RefusalResponse(
                message="I cannot answer this question as it's not covered in the provided textbook content.",
                reason="no_context_found",
                session_id=request.session_id
            )

        # Generate response using the retrieved context
        generation_start = time.time()
        response = generation_service.generate_response_from_context(
            question=request.question,
            context_chunks=relevant_chunks,
            session_id=request.session_id
        )
        generation_time = time.time() - generation_start

        # Log generation event
        log_generation_event(
            query=request.question,
            session_id=request.session_id,
            response_length=len(response.response),
            generation_time=generation_time,
            confidence_score=response.confidence
        )

        # Log the successful response
        logger.info(f"Successfully generated response for session {request.session_id}")

        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            retrieval_time=retrieval_time,
            generation_time=generation_time
        )

        return response

    except HTTPException as e:
        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            has_error=True
        )

        # Log the error
        log_error(
            error_type="HTTPException",
            error_message=str(e.detail) if hasattr(e, 'detail') else str(e),
            session_id=request.session_id,
            query=request.question if 'request' in locals() else "unknown"
        )

        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            has_error=True
        )

        # Log the error
        log_error(
            error_type="InternalServerError",
            error_message=str(e),
            session_id=request.session_id,
            query=request.question if 'request' in locals() else "unknown"
        )

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An internal error occurred while processing your request"
        )


@router.post("/selected-text", response_model=Union[ChatQueryResponse, RefusalResponse])
async def chat_selected_text(request: SelectedTextQueryRequest):
    """
    Handle a chat query with user-selected text context.
    This bypasses the vector database and uses only the provided selected text.

    Args:
        request: Selected text query request with question, selected text, and session information

    Returns:
        ChatQueryResponse with the answer based only on selected text
    """
    start_time = time.time()
    monitoring_service.metrics_collector.start_request()

    try:
        # Validate the incoming request
        if not validation_service.validate_question(request.question):
            monitoring_service.record_request(
                response_time=time.time() - start_time,
                has_error=True
            )
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid question provided"
            )

        # Log the query for analytics and debugging
        logger.info(f"Processing selected-text query for session {request.session_id}: {request.question[:100]}...")

        # For consistency with our new approach, we'll use book content only regardless of selected text
        # Retrieve relevant chunks based on the question
        retrieval_start = time.time()
        context_chunks = retrieval_service.retrieve_relevant_chunks(
            query=request.question,
            limit=10,  # Get up to 10 relevant chunks
            min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD  # Use configured threshold
        )
        retrieval_time = time.time() - retrieval_start

        # Log retrieval event
        log_retrieval_event(
            query=request.question,
            session_id=request.session_id,
            num_chunks_found=len(context_chunks),
            retrieval_time=retrieval_time,
            similarity_threshold=settings.MIN_SIMILARITY_THRESHOLD
        )

        # If no relevant chunks found, return a clear refusal response
        if not context_chunks:
            logger.info("No relevant context found for selected-text query request")

            response_time = time.time() - start_time
            monitoring_service.record_request(
                response_time=response_time,
                retrieval_time=retrieval_time
            )

            return RefusalResponse(
                message="I cannot answer this question as it's not covered in the provided textbook content.",
                reason="no_context_found",
                session_id=request.session_id
            )

        # Generate response based on the context
        generation_start = time.time()
        response = generation_service.generate_response_from_context(
            question=request.question,
            context_chunks=context_chunks,
            session_id=request.session_id  # Using the provided session ID
        )
        generation_time = time.time() - generation_start

        # Log generation event
        log_generation_event(
            query=request.question,
            session_id=request.session_id,
            response_length=len(response.response),
            generation_time=generation_time,
            confidence_score=response.confidence
        )

        # Log the successful response
        logger.info(f"Successfully generated response for selected-text query request")

        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            retrieval_time=retrieval_time,
            generation_time=generation_time
        )

        return response

    except HTTPException as e:
        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            has_error=True
        )

        # Log the error
        log_error(
            error_type="HTTPException",
            error_message=str(e.detail) if hasattr(e, 'detail') else str(e),
            session_id=request.session_id,
            query=request.question if 'request' in locals() else "unknown"
        )

        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            has_error=True
        )

        # Log the error
        log_error(
            error_type="InternalServerError",
            error_message=str(e),
            session_id=request.session_id,
            query=request.question if 'request' in locals() else "unknown"
        )

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An internal error occurred while processing your request"
        )


@router.post("/ask", response_model=Dict[str, str])
async def ask_question(question_data: Dict[str, str]):
    """
    Simple endpoint for frontend to ask questions.
    Expects JSON body: {"question": "your question here", "selectedText": "optional selected text"}

    Args:
        question_data: Dictionary containing the question and optionally selected text

    Returns:
        Dictionary with the answer
    """
    start_time = time.time()
    monitoring_service.metrics_collector.start_request()

    question = question_data.get("question", "").strip()
    # Ignore selectedText parameter and always use book content
    # selected_text = question_data.get("selectedText", "").strip() or question_data.get("selected_text", "").strip()

    if not question:
        monitoring_service.record_request(
            response_time=time.time() - start_time
        )
        return {"answer": "❌ Please provide a valid question."}

    if not validation_service.validate_question(question):
        monitoring_service.record_request(
            response_time=time.time() - start_time,
            has_error=True
        )
        return {"answer": "❌ Invalid question provided."}

    # Log the query for analytics and debugging
    logger.info(f"Processing frontend ask request: {question[:100]}...")

    try:
        # Always proceed with normal retrieval from book content only
        # Retrieve relevant chunks based on the question
        retrieval_start = time.time()
        context_chunks = retrieval_service.retrieve_relevant_chunks(
            query=question,
            limit=10,  # Get up to 10 relevant chunks
            min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD  # Use configured threshold
        )
        retrieval_time = time.time() - retrieval_start

        # Log retrieval event
        log_retrieval_event(
            query=question,
            session_id="frontend-session",
            num_chunks_found=len(context_chunks),
            retrieval_time=retrieval_time,
            similarity_threshold=settings.MIN_SIMILARITY_THRESHOLD
        )

        # If no relevant chunks found, return a clear refusal response
        if not context_chunks:
            logger.info("No relevant context found for frontend ask request")

            response_time = time.time() - start_time
            monitoring_service.record_request(
                response_time=response_time,
                retrieval_time=retrieval_time
            )

            return {"answer": "⚠️ This question is not covered in the textbook content."}

        # Generate response based on the context
        generation_start = time.time()
        response = generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id="frontend-session"  # Using a generic session ID for frontend requests
        )
        generation_time = time.time() - generation_start

        # Log generation event
        log_generation_event(
            query=question,
            session_id="frontend-session",
            response_length=len(response.response),
            generation_time=generation_time,
            confidence_score=response.confidence
        )

        # Log the successful response
        logger.info("Successfully generated response for frontend ask request")

        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            retrieval_time=retrieval_time,
            generation_time=generation_time
        )

        # Return only the answer for the frontend
        return {"answer": response.response}

    except Exception as e:
        response_time = time.time() - start_time
        monitoring_service.record_request(
            response_time=response_time,
            has_error=True
        )

        # Log the error
        log_error(
            error_type="InternalServerError",
            error_message=str(e),
            session_id="frontend-session",
            query=question
        )

        return {"answer": f"❌ Error processing the question: {str(e)}"}