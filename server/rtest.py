#!/usr/bin/env python
import json
from http.server import BaseHTTPRequestHandler, HTTPServer

class PumpRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        # Check the URL path
        if self.path == "/pump":
            content_length = int(self.headers['Content-Length'])  # Get size of the POST data
            post_data = self.rfile.read(content_length)  # Read the POST data

            try:
                # Parse the JSON data
                data = json.loads(post_data)
                duty = data.get("duty", None)

                if duty is not None and isinstance(duty, (int, float)) and 0 <= duty <= 100:
                    duty = int(duty)
                    print(f"Received valid POST request with duty: {duty} name: {data.get('name', 'indefined')}")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "success", "received_duty": duty}).encode())
                else:
                    # Invalid duty cycle
                    print("Invalid POST data:", data)
                    self.send_response(400)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "error", "message": "Invalid duty value"}).encode())
            except json.JSONDecodeError:
                # Invalid JSON
                print("Invalid JSON received")
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": "Invalid JSON"}).encode())
        else:
            # Invalid path
            self.send_response(404)
            self.end_headers()

def run_server():
    server_address = ('', 8080)  # Listen on all interfaces, port 8080
    httpd = HTTPServer(server_address, PumpRequestHandler)
    print("Server started on port 8080...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        httpd.server_close()

if __name__ == "__main__":
    run_server()

