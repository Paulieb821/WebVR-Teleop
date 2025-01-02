from flask import Flask, send_from_directory, redirect, url_for
import os
import logging

app = Flask(__name__, static_folder='client')

# Serve the index.html by default
@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')

# Serve other files from the 'client' folder
@app.route('/<path:filename>')
def serve_file(filename):
    return send_from_directory(app.static_folder, filename)

# Suppress Flask's default request logs
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

if __name__ == '__main__':
    # Path to the SSL certificate and key files (server.cert and server.key)
    cert_file = os.path.join('ssl', 'server.cert')
    key_file = os.path.join('ssl', 'server.key')
    
    # Run the Flask app with SSL (https)
    print('WebServer running at https://localhost:3000')
    app.run(host='0.0.0.0', port=3000, ssl_context=(cert_file, key_file))
