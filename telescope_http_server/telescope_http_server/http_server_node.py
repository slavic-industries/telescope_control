import http.server
import socketserver
import urllib.parse
import rclpy
from rclpy.node import Node
from telescope_interfaces.srv import RequestTargetData


PORT = 8000


class HttpRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        html = '''
        <html>
        <body>
        <h2>ROS 2 Service Call</h2>
        <form action="/" method="post">
          <label for="input">Enter a string:</label><br><br>
          <input type="text" id="input" name="input" value=""><br><br>
          <input type="submit" value="Submit">
        </form> 
        </body>
        </html>
        '''
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes(html, "utf8"))

    def do_POST(self):
        # Handle the form submission
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        post_data = urllib.parse.parse_qs(post_data.decode('utf-8'))

        input_value = post_data.get('input', [None])[0]
        if input_value:
            # Call the ROS 2 service with the input value
            client = self.server.node.create_client(
                RequestTargetData, 'request_target_data')
            request = RequestTargetData.Request()
            request.target_name = input_value

            # Retry mechanism with timeout
            max_attempts = 5
            attempts = 0
            service_found = False
            response_message = "Service call failed after 5 attempts."

            while attempts < max_attempts:
                if service_found:
                    break
                if client.wait_for_service(timeout_sec=1.0):
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self.server.node, future)
                    if future.result().success:
                        self.server.node.get_logger().info(
                            f"Target NAME: {future.result().target_name}")
                        self.server.node.get_logger().info(
                            f"Target RA: {future.result().ra}")
                        self.server.node.get_logger().info(
                            f"Target DEC: {future.result().dec}")
                        self.server.node.get_logger().info(
                            f"Target DIST: {future.result().dist}")
                        self.server.node.get_logger().info(
                            f"Target ALT: {future.result().alt}")
                        self.server.node.get_logger().info(
                            f"Target AZ: {future.result().az}")
                        service_found = True
                        response_message = f"Target NAME: {future.result().target_name}\r\nTarget RA: {future.result().ra}\r\nTarget DEC: {future.result().dec}\r\nTarget DIST: {future.result().dist}\r\nTarget ALT: {future.result().alt}\r\nTarget AZ: {future.result().az}"
                    else:
                        response_message = "Object cannot be found in the database."
                        self.server.node.get_logger().info(response_message)
                        service_found = True
                else:
                    self.server.node.get_logger().info(
                        f"Service not available, attempt {attempts + 1}/{max_attempts}")
                attempts += 1

        # Send response back to the browser
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes(
            f"<html><body><h2>{response_message}</h2><a href='/'>Go Back</a></body></html>", "utf8"))


class HTTPServerNode(Node):
    def __init__(self):
        super().__init__('simple_http_server')
        self.server = socketserver.TCPServer(("", PORT), HttpRequestHandler)
        self.server.node = self  # Pass the ROS node to the HTTP server
        self.get_logger().info(f"Starting HTTP server on port {PORT}")
        self.server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = HTTPServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
