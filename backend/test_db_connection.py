"""
Test script to verify the Neon Postgres database connection.
"""
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_db_connection():
    print("Testing Neon Postgres database connection...")
    
    try:
        # Import the settings and database client
        from src.config.settings import settings
        from src.database.postgres_client import postgres_client
        
        print(f"Database URL from settings: {settings.NEON_DATABASE_URL}")
        
        # Check if the database URL is properly set
        if not settings.NEON_DATABASE_URL or settings.NEON_DATABASE_URL == "":
            print("❌ ERROR: NEON_DATABASE_URL is not set in settings!")
            print("Please check your .env file and ensure NEON_DATABASE_URL is properly configured.")
            return False
        
        print("✅ NEON_DATABASE_URL is set in settings")
        
        # Test the connection
        try:
            health_status = postgres_client.health_check()
            if health_status:
                print("✅ Successfully connected to Neon Postgres database!")
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
    success = test_db_connection()
    if success:
        print("\n🎉 Database connection test PASSED!")
        print("The loading loop issue should be fixed.")
    else:
        print("\n❌ Database connection test FAILED!")
        print("Please check your database configuration.")