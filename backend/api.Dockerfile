FROM python:3.9-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY backend/api/requirements.txt .
RUN pip install -r requirements.txt

COPY backend/api ./
COPY frontend/dist ./static

EXPOSE 5000

CMD ["python", "app.py"]