FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV PYTHONUNBUFFERED=1
ENV MQTT_BROKER=mar-mosquitto
ENV MQTT_PORT=1883
ENV WS_PORT=8765

EXPOSE 8765

CMD ["python", "websocket_gateway.py"]
