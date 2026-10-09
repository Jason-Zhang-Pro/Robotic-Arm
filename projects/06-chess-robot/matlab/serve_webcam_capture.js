const http = require("http");
const fs = require("fs");
const path = require("path");

const root = __dirname;
const port = Number(process.env.PORT || 8765);

const server = http.createServer((req, res) => {
  const urlPath = req.url === "/" ? "/webcam_capture.html" : req.url;
  const filePath = path.join(root, decodeURIComponent(urlPath.split("?")[0]));
  if (!filePath.startsWith(root)) {
    res.writeHead(403);
    res.end("Forbidden");
    return;
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404);
      res.end("Not found");
      return;
    }
    const ext = path.extname(filePath).toLowerCase();
    const type = ext === ".html" ? "text/html" : ext === ".png" ? "image/png" : "text/plain";
    res.writeHead(200, { "Content-Type": type });
    res.end(data);
  });
});

server.listen(port, "127.0.0.1", () => {
  console.log(`Serving webcam capture page at http://127.0.0.1:${port}/`);
});
