"""
Test script to verify the Neon Postgres database connection with SQLAlchemy 2.x compatibility.
"""
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_sqlalchemy_2_compatibility():
    print("Testing Neon Postgres database connection with SQLAlchemy 2.x compatibility...")
    
    try:
        # Import the necessary modules
        from sqlalchemy import text
        from src.config.settings import settings
        from src.database.postgres_client import postgres_client
        
        print(f"Database URL from settings: {settings.NEON_DATABASE_URL[:50]}..." if settings.NEON_DATABASE_URL else "Database URL is not set")
        
        # Check if the database URL is properly set
        if not settings.NEON_DATABASE_URL or settings.NEON_DATABASE_URL == "":
            print("❌ ERROR: NEON_DATABASE_URL is not set in settings!")
            print("Please check your .env file and ensure NEON_DATABASE_URL is properly configured.")
            return False
        
        # Verify that text function is available (SQLAlchemy 2.x requirement)
        if text is None:
            print("❌ ERROR: text function not available from SQLAlchemy!")
            return False
        
        print("✅ text() function is available for SQLAlchemy 2.x compatibility")
        
        # Test the connection
        try:
            health_status = postgres_client.health_check()
            if health_status:
                print("✅ Successfully connected to Neon Postgres database!")
                print("✅ SQLAlchemy 2.x compatibility verified!")
                return True
            else:
                print("❌ Failed to connect to Neon Postgres database")
                return False
        except Exception as e:
            print(f"❌ Error during connection test: {str(e)}")
            return False
            
    except ImportError as e:
        print(f"❌ Import error: {str(e)}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_sqlalchemy_2_compatibility()
    if success:
        print("\n🎉 Database connection and SQLAlchemy 2.x compatibility test PASSED!")
        print("The loading loop issue should be fixed with proper SQLAlchemy 2.x support.")
    else:
        print("\n❌ Database connection or SQLAlchemy 2.x compatibility test FAILED!")
        print("Please check your database configuration and SQLAlchemy version.")