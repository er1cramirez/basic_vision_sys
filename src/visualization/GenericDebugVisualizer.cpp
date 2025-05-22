#include "GenericDebugVisualizer.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

GenericDebugVisualizer::GenericDebugVisualizer(int port, const std::string& title)
    : serverPort(port),
      interfaceTitle(title),
      jpegQuality(75),
      maxFrameWidth(800),
      maxFrameHeight(600),
      maxQueueSize(2),
      running(false),
      lastFrameId(-1),
      currentFps(0.0),
      systemStatus("Initializing...") {
    
    startTime = std::chrono::steady_clock::now();
    lastFrameTime = startTime;
}

GenericDebugVisualizer::~GenericDebugVisualizer() {
    stop();
}

void GenericDebugVisualizer::start() {
    if (running.load()) {
        return;
    }
    
    running = true;
    serverThread = std::thread(&GenericDebugVisualizer::serverLoop, this);
    
    std::cout << "Generic Debug Visualizer started" << std::endl;
    std::cout << "→ Web Interface: http://localhost:" << serverPort << std::endl;
    std::cout << "→ Title: " << interfaceTitle << std::endl;
}

void GenericDebugVisualizer::stop() {
    if (!running.load()) {
        return;
    }
    
    running = false;
    frameCondition.notify_all();
    
    if (serverThread.joinable()) {
        serverThread.join();
    }
    
    std::cout << "Generic Debug Visualizer stopped" << std::endl;
}

bool GenericDebugVisualizer::isRunning() const {
    return running.load();
}

void GenericDebugVisualizer::updateFrame(const cv::Mat& frame, int frameId) {
    if (!running.load() || frame.empty()) {
        return;
    }
    
    // Update frame ID if provided
    if (frameId >= 0) {
        lastFrameId = frameId;
    }
    
    // Update FPS counter
    updateFpsCounter();
    
    // Queue frame for processing
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        
        // Keep queue size manageable
        while (frameQueue.size() >= maxQueueSize) {
            frameQueue.pop();
        }
        
        frameQueue.push(frame.clone());
    }
    
    frameCondition.notify_one();
}

void GenericDebugVisualizer::setData(const std::string& key, const std::string& value) {
    std::lock_guard<std::mutex> lock(dataMutex);
    dataMap[key] = value;
}

void GenericDebugVisualizer::setData(const std::string& key, double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    setData(key, oss.str());
}

void GenericDebugVisualizer::setData(const std::string& key, int value) {
    setData(key, std::to_string(value));
}

void GenericDebugVisualizer::setData(const std::string& key, bool value) {
    setData(key, value ? "true" : "false");
}

void GenericDebugVisualizer::setData(const std::map<std::string, std::string>& data) {
    std::lock_guard<std::mutex> lock(dataMutex);
    for (const auto& pair : data) {
        dataMap[pair.first] = pair.second;
    }
}

void GenericDebugVisualizer::removeData(const std::string& key) {
    std::lock_guard<std::mutex> lock(dataMutex);
    dataMap.erase(key);
}

void GenericDebugVisualizer::clearData() {
    std::lock_guard<std::mutex> lock(dataMutex);
    dataMap.clear();
}

void GenericDebugVisualizer::setStatus(const std::string& status) {
    std::lock_guard<std::mutex> lock(dataMutex);
    systemStatus = status;
}

int GenericDebugVisualizer::getPort() const {
    return serverPort;
}

void GenericDebugVisualizer::setJpegQuality(int quality) {
    jpegQuality = std::max(1, std::min(100, quality));
}

void GenericDebugVisualizer::setMaxFrameSize(int width, int height) {
    maxFrameWidth = width;
    maxFrameHeight = height;
}

void GenericDebugVisualizer::setTitle(const std::string& title) {
    interfaceTitle = title;
}

void GenericDebugVisualizer::updateFpsCounter() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrameTime).count();
    
    if (elapsed > 0) {
        double instantFps = 1000.0 / elapsed;
        // Simple exponential moving average
        currentFps = 0.8 * currentFps + 0.2 * instantFps;
    }
    
    lastFrameTime = now;
}

void GenericDebugVisualizer::processFrameQueue() {
    cv::Mat frame;
    
    // Get frame from queue
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (frameQueue.empty()) {
            return;
        }
        
        frame = frameQueue.front();
        frameQueue.pop();
    }
    
    if (frame.empty()) {
        return;
    }
    
    // Resize if needed
    cv::Mat processedFrame;
    if (frame.cols > maxFrameWidth || frame.rows > maxFrameHeight) {
        double scale = std::min(
            static_cast<double>(maxFrameWidth) / frame.cols,
            static_cast<double>(maxFrameHeight) / frame.rows
        );
        cv::resize(frame, processedFrame, cv::Size(), scale, scale, cv::INTER_AREA);
    } else {
        processedFrame = frame;
    }
    
    // Compress to JPEG
    std::vector<uchar> jpegBuffer;
    std::vector<int> compressionParams = {cv::IMWRITE_JPEG_QUALITY, jpegQuality};
    
    if (cv::imencode(".jpg", processedFrame, jpegBuffer, compressionParams)) {
        std::lock_guard<std::mutex> lock(currentFrameMutex);
        currentJpeg = jpegBuffer;
        frameCounter++;
    }
}

void GenericDebugVisualizer::serverLoop() {
    httplib::Server server;
    
    // Serve main HTML page
    server.Get("/", [this](const httplib::Request&, httplib::Response& res) {
        res.set_content(generateHtmlPage(), "text/html");
    });
    
    // Serve video stream
    server.Get("/stream", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_header("Pragma", "no-cache");
        res.set_header("Expires", "0");
        res.set_header("Content-Type", "image/jpeg");
        
        std::lock_guard<std::mutex> lock(currentFrameMutex);
        if (!currentJpeg.empty()) {
            res.set_content((const char*)currentJpeg.data(), currentJpeg.size(), "image/jpeg");
        } else {
            // Generate placeholder image
            cv::Mat placeholder(240, 320, CV_8UC3, cv::Scalar(64, 64, 64));
            cv::putText(placeholder, "No Video Feed", cv::Point(80, 120), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(placeholder, "Waiting for frames...", cv::Point(50, 160), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
            
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpegQuality};
            cv::imencode(".jpg", placeholder, buffer, params);
            res.set_content((const char*)buffer.data(), buffer.size(), "image/jpeg");
        }
    });
    
    // API endpoints
    server.Get("/api/data", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Content-Type", "application/json");
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_content(generateDataJson(), "application/json");
    });
    
    server.Get("/api/system", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Content-Type", "application/json");
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_content(generateSystemInfo(), "application/json");
    });
    
    server.Get("/api/frame_count", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Content-Type", "application/json");
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_content("{\"count\": " + std::to_string(frameCounter.load()) + "}", "application/json");
    });
    
    // Start HTTP server in background thread
    std::thread serverRunner([&]() {
        server.listen("0.0.0.0", serverPort);
    });
    
    // Main processing loop
    while (running.load()) {
        // Wait for frames
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait_for(lock, std::chrono::milliseconds(50));
        }
        
        // Process any queued frames
        processFrameQueue();
        
        // Small sleep to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    // Cleanup
    server.stop();
    if (serverRunner.joinable()) {
        serverRunner.join();
    }
}

std::string GenericDebugVisualizer::generateHtmlPage() {
    std::stringstream html;
    
    html << R"(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>)" << interfaceTitle << R"(</title>
    <style>
        :root {
            --bg-dark: #0d1117;
            --bg-secondary: #161b22;
            --bg-tertiary: #21262d;
            --border-color: #30363d;
            --text-primary: #f0f6fc;
            --text-secondary: #8b949e;
            --accent-blue: #58a6ff;
            --accent-green: #3fb950;
            --accent-red: #f85149;
            --accent-yellow: #d29922;
        }
        
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif;
            background: var(--bg-dark);
            color: var(--text-primary);
            line-height: 1.5;
        }
        
        .header {
            background: var(--bg-secondary);
            border-bottom: 1px solid var(--border-color);
            padding: 1rem 2rem;
        }
        
        .header h1 {
            color: var(--accent-blue);
            font-size: 1.5rem;
            font-weight: 600;
        }
        
        .container {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 1.5rem;
            padding: 1.5rem;
            max-width: 1400px;
            margin: 0 auto;
        }
        
        .card {
            background: var(--bg-secondary);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            overflow: hidden;
        }
        
        .card-header {
            background: var(--bg-tertiary);
            padding: 0.75rem 1rem;
            border-bottom: 1px solid var(--border-color);
            font-weight: 600;
            font-size: 0.875rem;
        }
        
        .card-body {
            padding: 1rem;
        }
        
        .video-container {
            background: #000;
            position: relative;
            border-radius: 4px;
            overflow: hidden;
        }
        
        .video-stream {
            width: 100%;
            height: auto;
            display: block;
        }
        
        .video-overlay {
            position: absolute;
            top: 0.5rem;
            right: 0.5rem;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 0.25rem 0.5rem;
            border-radius: 4px;
            font-size: 0.75rem;
            font-family: monospace;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
            gap: 0.75rem;
            margin-bottom: 1rem;
        }
        
        .stat-card {
            background: var(--bg-tertiary);
            border: 1px solid var(--border-color);
            border-radius: 4px;
            padding: 0.75rem;
            text-align: center;
        }
        
        .stat-value {
            display: block;
            font-size: 1.25rem;
            font-weight: bold;
            color: var(--accent-blue);
            font-family: monospace;
        }
        
        .stat-label {
            font-size: 0.75rem;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        
        .data-table {
            width: 100%;
        }
        
        .data-table th,
        .data-table td {
            text-align: left;
            padding: 0.5rem;
            border-bottom: 1px solid var(--border-color);
        }
        
        .data-table th {
            background: var(--bg-tertiary);
            font-weight: 600;
            font-size: 0.875rem;
        }
        
        .data-table td {
            font-family: monospace;
            font-size: 0.8rem;
        }
        
        .data-key {
            color: var(--text-secondary);
            max-width: 120px;
            word-wrap: break-word;
        }
        
        .data-value {
            color: var(--text-primary);
        }
        
        .status-online { color: var(--accent-green); }
        .status-warning { color: var(--accent-yellow); }
        .status-error { color: var(--accent-red); }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
                padding: 1rem;
            }
        }
        
        .refresh-indicator {
            display: inline-block;
            width: 8px;
            height: 8px;
            background: var(--accent-green);
            border-radius: 50%;
            margin-left: 0.5rem;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.3; }
            100% { opacity: 1; }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>)" << interfaceTitle << R"( <span class="refresh-indicator"></span></h1>
    </div>
    
    <div class="container">
        <div class="main-content">
            <div class="card">
                <div class="card-header">
                    Live Video Feed
                </div>
                <div class="card-body" style="padding: 0;">
                    <div class="video-container">
                        <img id="video-stream" class="video-stream" src="/stream" alt="Video Stream">
                        <div class="video-overlay">
                            Frame: <span id="frame-count">0</span> | 
                            FPS: <span id="fps-display">0</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <div class="card-header">System Statistics</div>
                <div class="card-body">
                    <div class="stats-grid" id="stats-grid">
                        <div class="stat-card">
                            <span class="stat-value" id="stat-fps">--</span>
                            <div class="stat-label">FPS</div>
                        </div>
                        <div class="stat-card">
                            <span class="stat-value" id="stat-frames">--</span>
                            <div class="stat-label">Frames</div>
                        </div>
                        <div class="stat-card">
                            <span class="stat-value" id="stat-uptime">--</span>
                            <div class="stat-label">Uptime</div>
                        </div>
                        <div class="stat-card">
                            <span class="stat-value" id="stat-status">--</span>
                            <div class="stat-label">Status</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="sidebar">
            <div class="card">
                <div class="card-header">Data Monitor</div>
                <div class="card-body">
                    <table class="data-table" id="data-table">
                        <thead>
                            <tr>
                                <th>Key</th>
                                <th>Value</th>
                            </tr>
                        </thead>
                        <tbody id="data-table-body">
                            <tr>
                                <td colspan="2" style="text-align: center; color: var(--text-secondary);">
                                    No data available
                                </td>
                            </tr>
                        </tbody>
                    </table>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let lastFrameCount = 0;
        let lastFrameTime = Date.now();
        
        function updateVideo() {
            fetch('/api/frame_count')
                .then(response => response.json())
                .then(data => {
                    const currentCount = data.count;
                    if (currentCount !== lastFrameCount) {
                        // Update frame counter
                        document.getElementById('frame-count').textContent = currentCount;
                        document.getElementById('stat-frames').textContent = currentCount;
                        
                        // Calculate FPS
                        const now = Date.now();
                        if (lastFrameCount > 0) {
                            const fps = Math.round(1000 / (now - lastFrameTime));
                            document.getElementById('fps-display').textContent = fps;
                            document.getElementById('stat-fps').textContent = fps;
                        }
                        
                        // Update video stream
                        document.getElementById('video-stream').src = '/stream?' + Date.now();
                        
                        lastFrameCount = currentCount;
                        lastFrameTime = now;
                    }
                })
                .catch(error => console.error('Error updating video:', error));
        }
        
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    const tableBody = document.getElementById('data-table-body');
                    
                    if (Object.keys(data).length === 0) {
                        tableBody.innerHTML = '<tr><td colspan="2" style="text-align: center; color: var(--text-secondary);">No data available</td></tr>';
                        return;
                    }
                    
                    let html = '';
                    for (const [key, value] of Object.entries(data)) {
                        html += `<tr>
                            <td class="data-key">${key}</td>
                            <td class="data-value">${value}</td>
                        </tr>`;
                    }
                    tableBody.innerHTML = html;
                })
                .catch(error => console.error('Error updating data:', error));
        }
        
        function updateSystem() {
            fetch('/api/system')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('stat-status').textContent = data.status || 'Unknown';
                    
                    if (data.uptime !== undefined) {
                        const hours = Math.floor(data.uptime / 3600);
                        const minutes = Math.floor((data.uptime % 3600) / 60);
                        const seconds = data.uptime % 60;
                        document.getElementById('stat-uptime').textContent = 
                            `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
                    }
                })
                .catch(error => console.error('Error updating system:', error));
        }
        
        // Update intervals
        setInterval(updateVideo, 100);    // 10 FPS for video updates
        setInterval(updateData, 500);     // 2 Hz for data updates
        setInterval(updateSystem, 1000);  // 1 Hz for system updates
        
        // Initial updates
        updateVideo();
        updateData();
        updateSystem();
    </script>
</body>
</html>)";
    
    return html.str();
}

std::string GenericDebugVisualizer::generateDataJson() {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    std::stringstream json;
    json << "{";
    
    bool first = true;
    for (const auto& pair : dataMap) {
        if (!first) json << ",";
        json << "\"" << pair.first << "\":\"" << pair.second << "\"";
        first = false;
    }
    
    json << "}";
    return json.str();
}

std::string GenericDebugVisualizer::generateSystemInfo() {
    auto now = std::chrono::steady_clock::now();
    auto uptime = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    
    std::stringstream json;
    json << "{";
    json << "\"uptime\":" << uptime << ",";
    json << "\"fps\":" << std::fixed << std::setprecision(1) << currentFps << ",";
    json << "\"frameCount\":" << frameCounter.load() << ",";
    
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        json << "\"status\":\"" << systemStatus << "\"";
    }
    
    json << "}";
    return json.str();
}