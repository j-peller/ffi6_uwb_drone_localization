from datetime import datetime
from flask import Flask, render_template
from flask_sock import Sock
import json

app = Flask(__name__)
sock = Sock(app)

log_file = datetime.now().strftime('%Y%m%d_%H%M%S_messages.log')

# Liste der verbundenen Clients
connected_clients = set()

@app.route('/')
def index():
    try:
        with open(log_file, 'r') as file:
            lines = file.readlines()
        lines = [line.strip().split(' ', 3) for line in reversed(lines)]
        return render_template('logs.html', lines=lines)
    except Exception as e:
        return f'Error reading log: {str(e)}', 500

@sock.route('/ws')
def websocket_endpoint(ws):
    # Client hinzufügen
    connected_clients.add(ws)

    try:
        while True:
            message = ws.receive()
            if message:
                try:
                    # JSON-Daten parsen
                    data = json.loads(message)
                    esp_id = data.get("esp_id", "???")
                    msg = data.get("message", "???")
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                    # Log-Nachricht erstellen
                    log_message = f'{timestamp} {esp_id} {msg}\n'
                    print(log_message)

                    # In Datei speichern
                    with open(log_file, 'a') as file:
                        file.write(log_message)

                    # Nachricht im JSON-Format vorbereiten
                    broadcast_message = json.dumps({
                        "timestamp": timestamp,
                        "id": esp_id,
                        "message": msg
                    })

                    # An alle verbundenen Clients senden
                    for client in connected_clients.copy():
                        try:
                            client.send(broadcast_message)
                        except Exception:
                            connected_clients.remove(client)

                except json.JSONDecodeError:
                    print("Error: Unknown JSON format")
                    ws.send("Error: Unknown JSON format")
            else:
                break
    finally:
        # Client entfernen, wenn die Verbindung geschlossen wird
        connected_clients.remove(ws)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
