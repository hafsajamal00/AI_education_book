# Deployment Configuration for VLA Module

## Environment Variables

The following environment variables are required for deployment:

```bash
# OpenAI API key for Whisper and LLM functionality
OPENAI_API_KEY=your_openai_api_key_here

# ROS 2 Domain ID (default is 0, change for multi-robot setups)
ROS_DOMAIN_ID=0

# Web server configuration
PORT=3000
HOST=0.0.0.0

# Logging configuration
LOG_LEVEL=info
LOG_DIR=./logs

# Performance monitoring
MONITORING_ENABLED=true
PERFORMANCE_LOG_INTERVAL=5  # seconds
```

## Docker Configuration

### Dockerfile

```dockerfile
FROM node:18-alpine

# Install Python and other dependencies
RUN apk add --no-cache python3 py3-pip

# Set working directory
WORKDIR /app

# Copy package files
COPY package*.json ./

# Install Node.js dependencies
RUN npm install

# Copy Docusaurus source
COPY frontend_book/ ./frontend_book/

# Build the Docusaurus site
WORKDIR /app/frontend_book
RUN npm run build

# Expose port
EXPOSE 3000

# Start the server
CMD ["npx", "docusaurus", "start", "--port", "3000", "--host", "0.0.0.0"]
```

### docker-compose.yml

```yaml
version: '3.8'

services:
  vla-documentation:
    build: .
    ports:
      - "3000:3000"
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
    volumes:
      - ./logs:/app/logs
    restart: unless-stopped
    networks:
      - vla-network

  ros2-vla-backend:
    build:
      context: .
      dockerfile: ros2_vla_integration/Dockerfile
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw  # For GUI if needed
    devices:
      - /dev/snd:/dev/snd  # For audio input
    privileged: true
    network_mode: host  # ROS2 requires host network for discovery
    restart: unless-stopped

networks:
  vla-network:
    driver: bridge
```

## Kubernetes Configuration (Optional)

### vla-module-deployment.yaml

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: vla-documentation
  labels:
    app: vla-documentation
spec:
  replicas: 2
  selector:
    matchLabels:
      app: vla-documentation
  template:
    metadata:
      labels:
        app: vla-documentation
    spec:
      containers:
      - name: vla-documentation
        image: vla-documentation:latest
        ports:
        - containerPort: 3000
        env:
        - name: OPENAI_API_KEY
          valueFrom:
            secretKeyRef:
              name: vla-secrets
              key: openai-api-key
        - name: PORT
          value: "3000"
        volumeMounts:
        - name: logs
          mountPath: /app/logs
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
      volumes:
      - name: logs
        emptyDir: {}
---
apiVersion: v1
kind: Service
metadata:
  name: vla-documentation-service
spec:
  selector:
    app: vla-documentation
  ports:
    - protocol: TCP
      port: 80
      targetPort: 3000
  type: LoadBalancer
```

## Production Deployment Steps

1. **Environment Setup**
   ```bash
   # Create environment file
   cp .env.example .env
   # Edit .env with your actual values
   vim .env
   ```

2. **Build the Documentation**
   ```bash
   cd frontend_book
   npm run build
   ```

3. **Deploy with Docker**
   ```bash
   # Build and start services
   docker-compose up -d
   ```

4. **Verify Deployment**
   ```bash
   # Check if services are running
   docker-compose ps
   
   # Check logs
   docker-compose logs -f
   ```

## Performance Monitoring Configuration

The system includes performance monitoring for key metrics:

- Voice recognition response time
- LLM processing time
- Action execution time
- System resource usage

### Monitoring Dashboard Setup

1. The system outputs metrics to standard logs in JSON format
2. These can be collected by monitoring tools like Prometheus
3. Visualization can be done with Grafana dashboards

### Alerting Configuration

Set up alerts for:

- Response time exceeding thresholds (>3 seconds)
- High error rates (>5% of requests)
- Resource utilization >80%
- Safety constraint violations

## Security Considerations

- API keys should be stored as secrets, not in code
- Implement rate limiting to prevent abuse
- Validate all user inputs
- Use HTTPS in production
- Regularly update dependencies