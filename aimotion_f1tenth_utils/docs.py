"""Utiliy script to open the documentation of the package in the browser."""

import webbrowser
import http.server
import os
import atexit

def open_docs_wondow():
    webbrowser.open('http://localhost:8000')

def start_server():

    httpd = http.server.HTTPServer(('localhost', 8000), http.server.SimpleHTTPRequestHandler)
    httpd.serve_forever()

def open_docs():
    # move to the docs folder
    docs_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs",'build','html')
    os.chdir(docs_path)
    open_docs_wondow()
    try:
        start_server()
    except KeyboardInterrupt:
        print("Docuemntation closed!")


if __name__ == "__main__":
    open_docs()