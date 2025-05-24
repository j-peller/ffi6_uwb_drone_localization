from datetime import datetime
from flask import Flask, render_template
from flask_sock import Sock
import json
import os

app = Flask(__name__)
sock = Sock(app)

LOG_DIR = os.path.dirname(os.path.abspath(__file__))
log_file = os.path.join(LOG_DIR,datetime.now().strftime('%Y%m%d_%H%M%S_messages.log'))

print(f"[INFO] Logging to: {log_file}")
with open(log_file, 'a') as f:
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    f.write(f"{timestamp} [LOG] Server started\n")

# Liste der verbundenen Clients
connected_loggers = set()
connected_views = set()

logger_ids = {}

def removeLogger(ws):
    logger_ids.pop(ws, 'None')
    connected_loggers.remove(ws)
    send_logger_list_to_views()

def send_logger_list_to_views():
    ids = list(set(logger_ids.values()))
    message = json.dumps({
        "type": "logger_list",
        "data": ids
    })
    for client in connected_views.copy():
        try:
            client.send(message)
        except Exception:
            connected_views.remove(client)

@app.route('/')
def index():
    try:
        with open(log_file, 'r') as file:
            lines = file.readlines()
        lines = [line.strip().split(' ', 3) for line in reversed(lines)]
        return render_template('logs.html', lines=lines)
    except Exception as e:
        return f'Error reading log: {str(e)}', 500

@sock.route('/ws-logger')
def logger_websocket_endpoint(ws):
    connected_loggers.add(ws)
    try:
        while True:
            message = ws.receive()
            print(f"message {message}\n")
            if not message:
                break
            try:
                logger_id = logger_ids.get(ws, "unknown")
                data = json.loads(message)
                msg_type = data.get("type", "???")
                msg_data = data.get("data", "???")
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] # time in milliseconds accuracy

                if msg_type == "id":
                    logger_ids[ws] = str(msg_data)
                    print(f"[INFO] Logger ID set: {msg_data}")
                    send_logger_list_to_views()  # Update Views mit aktueller Loggerliste
                elif msg_type == "log":
                    
                    log_message = f'{timestamp} {logger_id} {msg_data}\n'
                    print(log_message)

                    with open(log_file, 'a') as file:
                        file.write(log_message)

                    broadcast_message = json.dumps({
                        "timestamp": timestamp,
                        "id": logger_id,
                        "message": msg_data
                    })

                    # An Views weiterleiten
                    for client in connected_views.copy():
                        try:
                            client.send(broadcast_message)
                        except Exception:
                            connected_views.remove(client)
                            
            except json.JSONDecodeError:
                ws.send("Error: Invalid JSON")
    finally:
        removeLogger(ws)


@sock.route('/ws-view')
def view_websocket_endpoint(ws):
    # Client hinzufügen
    connected_views.add(ws)
    send_logger_list_to_views()
    try:
        while True:
            message = ws.receive()
            if not message:
                break
            try:
                data = json.loads(message)
                target_id = data.get("target_id")
                msg = data.get("message")
                if not target_id or not msg:
                    continue

                for logger in connected_loggers.copy():
                    try:
                        logger.send(json.dumps({
                            "from": "desktop",
                            "to": target_id,
                            "message": msg
                        }))
                    except Exception:
                        removeLogger(ws);
            except json.JSONDecodeError:
                ws.send("Error: Invalid JSON")
    finally:
        connected_views.remove(ws)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
