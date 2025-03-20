FROM python:3.12-alpine

WORKDIR /code

# Copy requirements and install
COPY ./req.txt .
RUN python -m pip install --no-cache-dir --upgrade -r req.txt \
    && rm req.txt

# Install curl
RUN apk add --no-cache curl

# Download custom redis communication handler
RUN curl -L -o ./redis_communication.py https://raw.githubusercontent.com/SSRS-Innovation/redis-communication/refs/heads/main/src/redis_communication.py

# Copy all source files
COPY . .

ENTRYPOINT ["python", "missionhandler.py"]