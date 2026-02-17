FROM python:3.11-slim

# Install system dependencies for MuJoCo (OSMesa) and pythonocc
RUN apt-get update && apt-get install -y --no-install-recommends \
    libosmesa6-dev \
    libgl1-mesa-dri \
    libglu1-mesa \
    libglfw3 \
    libglew-dev \
    libxrandr2 \
    patchelf \
    gcc \
    g++ \
    curl \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV MUJOCO_GL=osmesa
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/app

EXPOSE 8000

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
