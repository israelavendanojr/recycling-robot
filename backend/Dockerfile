FROM python:3.10-slim

WORKDIR /app

# System deps for OpenCV + ML + healthcheck
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install backend Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy backend code
COPY . .

EXPOSE 8000

# Run Flask backend
CMD ["flask", "--app", "app.py", "run", "--host=0.0.0.0", "--port=8000"]