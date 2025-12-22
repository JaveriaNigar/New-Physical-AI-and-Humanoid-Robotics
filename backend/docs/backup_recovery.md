# Backup and Recovery Procedures for RAG Chatbot Backend

## Overview
This document outlines the backup and recovery procedures for the RAG Chatbot backend system, including the vector database (Qdrant), metadata database (Postgres), and application configuration.

## Backup Procedures

### 1. Qdrant Vector Database Backup
Qdrant Cloud provides built-in backup capabilities:

1. **Configure automatic backups** in the Qdrant Cloud dashboard:
   - Navigate to your cluster settings
   - Enable automatic daily backups
   - Set retention policy (recommended: 30 days)

2. **Manual backup** (if needed):
   - Use Qdrant's snapshot functionality via the API
   - For local Qdrant: copy the entire storage directory

3. **Backup verification**:
   - Verify backup creation in the Qdrant Cloud dashboard
   - Test restore on a separate cluster periodically

### 2. Postgres Metadata Database Backup
For Neon Postgres, use the following approach:

1. **Use Neon's built-in branching**:
   - Create a long-running branch specifically for backup purposes
   - Set appropriate retention policies

2. **Manual backup** (if needed):
   ```bash
   # Using pg_dump for manual backup
   pg_dump --host=your-neon-host --port=5432 --username=your-username --dbname=your-db-name --file=backup.sql
   ```

3. **Backup schedule**:
   - Daily automated backups via Neon's settings
   - Weekly full backup verification

### 3. Application Configuration Backup
Backup the following configuration files and settings:

1. **Environment variables**:
   - Create a separate encrypted backup of `.env` files
   - Store API keys and secrets securely using a secrets management system

2. **Application code**:
   - Maintain version control with Git
   - Tag production releases appropriately

3. **Docker configurations**:
   - Backup `Dockerfile` and `docker-compose.yml` files
   - Store in version control

## Recovery Procedures

### 1. Qdrant Recovery
In case of data loss in Qdrant:

1. **Contact Qdrant Cloud support** if using cloud service
2. **Use the backup/restore functionality** in the dashboard
3. **Recreate the collection** with the same configuration:
   ```python
   # Recreate collection with same vector size and configuration
   client.create_collection(
       collection_name="textbook-content",
       vectors_config=VectorParams(size=768, distance=Distance.COSINE)
   )
   ```
4. **Reindex all textbook content** after recovery
5. **Validate retrieval functionality** after recovery

### 2. Postgres Recovery
For Postgres recovery:

1. **Use Neon's point-in-time recovery** feature
2. **Restore from branch** if using Neon's branching
3. **Apply manual backup** if needed:
   ```bash
   psql --host=your-neon-host --port=5432 --username=your-username --dbname=your-db-name --file=backup.sql
   ```
4. **Verify data integrity** after restore
5. **Update application connection settings** if needed

### 3. Application Recovery
For application recovery:

1. **Deploy from latest stable version** in Git
2. **Restore environment configuration**
3. **Verify all services are operational**:
   - Health checks
   - External API connections
   - Database connections
4. **Test all endpoints** manually after recovery

## Recovery Scenarios

### Scenario 1: Partial Data Loss in Qdrant
- Impact: Some textbook content unavailable
- Recovery steps:
  1. Identify the affected content
  2. Restore from Qdrant backup
  3. Reindex affected textbook sections
  4. Verify retrieval for affected content

### Scenario 2: Complete Qdrant Failure
- Impact: No textbook content available
- Recovery steps:
  1. Contact Qdrant support immediately
  2. Set up new Qdrant cluster if needed
  3. Restore from latest backup
  4. Reindex all textbook content
  5. Thoroughly test retrieval functionality

### Scenario 3: Postgres Metadata Issues
- Impact: Chunk metadata, session tracking unavailable
- Recovery steps:
  1. Restore from Neon backup
  2. Verify metadata integrity
  3. Reconnect to Qdrant vectors
  4. Test traceability features

### Scenario 4: Application Server Failure
- Impact: API unavailable
- Recovery steps:
  1. Deploy application to backup server
  2. Verify all configurations
  3. Test API endpoints
  4. Redirect traffic to backup server

## Testing and Validation

### Regular Testing
- Monthly: Test backup restoration in staging environment
- Quarterly: Full disaster recovery drill
- Annually: Review and update procedures

### Validation Checks
- Verify backup integrity using checksums
- Test retrieval and generation functionality after recovery
- Validate security configurations post-recovery
- Confirm all logging and monitoring systems operational

## Contact Information
- Qdrant Support: [Qdrant Cloud support contact]
- Neon Postgres Support: [Neon support contact]
- Application Team: [Team contact information]

## Revision History
- v1.0: Initial backup and recovery procedures
- v1.1: Added Neon-specific procedures
- v1.2: Updated with Qdrant Cloud procedures