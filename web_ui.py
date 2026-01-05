#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RASPBOT V2 - Web UI Server (FIXED)
Serves templates folder with NO CACHE headers
"""

from flask import Flask, send_from_directory, request  # ? ADDED request!
import os
from datetime import datetime

app = Flask(__name__, static_folder='templates', static_url_path='')

# ============================================================================
# DISABLE CACHE - Force fresh files every time
# ============================================================================

@app.after_request
def add_no_cache_headers(response):
    """Disable all caching - force fresh files"""
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

# ============================================================================
# ROUTES
# ============================================================================

@app.route('/')
def index():
    """Serve index.html"""
    try:
        file_path = os.path.join('templates', 'index.html')
        if not os.path.exists(file_path):
            return f"ERROR: index.html not found at {file_path}", 404
        
        print(f"[SERVE] index.html ({os.path.getsize(file_path)} bytes)")
        return send_from_directory('templates', 'index.html')
    except Exception as e:
        print(f"[ERROR] {e}")
        return f"ERROR: {str(e)}", 500

@app.route('/<path:path>')
def serve_static(path):
    """Serve static files (CSS, JS, images)"""
    try:
        file_path = os.path.join('templates', path)
        
        if not os.path.exists(file_path):
            print(f"[404] File not found: {path}")
            return f"File not found: {path}", 404
        
        size = os.path.getsize(file_path)
        print(f"[SERVE] {path} ({size} bytes)")
        
        return send_from_directory('templates', path)
    except Exception as e:
        print(f"[ERROR] {e}")
        return f"ERROR: {str(e)}", 500

@app.errorhandler(404)
def not_found(e):
    """Custom 404"""
    return f"Page not found: {request.path}", 404  # ? NOW request is defined!

@app.errorhandler(500)
def server_error(e):
    """Custom 500"""
    return f"Server error: {str(e)}", 500

# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    print("\n" + "="*70)
    print("?? RASPBOT V2 - WEB UI SERVER (FIXED)")
    print("="*70)
    
    # Check files exist
    files = ['index.html', 'style.css', 'script.js']
    print(f"\n?? Templates folder: {os.path.abspath('templates')}")
    print(f"\n? File check:")
    
    for fname in files:
        fpath = os.path.join('templates', fname)
        if os.path.exists(fpath):
            size = os.path.getsize(fpath)
            print(f"   ? {fname} ({size} bytes)")
        else:
            print(f"   ? {fname} NOT FOUND!")
    
    print(f"\n?? Web UI: http://localhost:3000")
    print(f"?? Logging all requests with file sizes")
    print(f"?? Caching: DISABLED (fresh files every request)")
    print(f"\n" + "="*70 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=3000, debug=False, use_reloader=False)  # ? Changed to 0.0.0.0
    except KeyboardInterrupt:
        print("\n??  Shutdown")
    except Exception as e:
        print(f"\n? Error: {e}")