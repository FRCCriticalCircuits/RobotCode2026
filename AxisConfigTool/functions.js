// functions.js

(function() {
    const canvas = document.getElementById("graph");
    const ctx = canvas.getContext("2d");

    let points = [
        {x:0, y:0},
        {x:100, y:100}
    ];

    let selectedPoint = null;
    let draggingPoint = null;

    function sortPoints() {
        points.sort((a,b) => a.x - b.x);
    }

    function draw() {
        ctx.clearRect(0,0,canvas.width,canvas.height);

        // 绘制网格
        ctx.strokeStyle = "#333";
        for (let i = 0; i <= 10; i++) {
            let x = i * canvas.width / 10;
            let y = i * canvas.height / 10;
            ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,canvas.height); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(canvas.width,y); ctx.stroke();
        }

        // 绘制曲线
        ctx.strokeStyle = "#00ff88";
        ctx.beginPath();
        for (let i = 0; i < points.length; i++) {
            let px = points[i].x / 100 * canvas.width;
            let py = canvas.height - (points[i].y / 100 * canvas.height);
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();

        // 绘制控制点
        for (let p of points) {
            let px = p.x / 100 * canvas.width;
            let py = canvas.height - (p.y / 100 * canvas.height);

            ctx.beginPath();
            ctx.arc(px, py, 6, 0, Math.PI * 2);
            ctx.fillStyle = (p === selectedPoint) ? "#00ff88" : "#ff5555";
            ctx.fill();
        }
    }

    function updateTable() {
        const tbody = document.querySelector("#pointsTable tbody");
        tbody.innerHTML = "";

        points.forEach((p, index) => {
            const row = document.createElement("tr");
            if (p === selectedPoint) row.classList.add("selectedRow");

            row.innerHTML = `
                <td>${index}</td>
                <td>${p.x.toFixed(2)}</td>
                <td>${p.y.toFixed(2)}</td>
            `;

            row.onclick = () => {
                selectedPoint = p;
                updatePanel();
                draw();
                updateTable();
            };

            tbody.appendChild(row);
        });
    }

    function addPoint() {
        points.push({x:50, y:50});
        sortPoints();
        updateTable();
        draw();
    }

    canvas.addEventListener("mousedown", e => {
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;

        for (let p of points) {
            let px = p.x / 100 * canvas.width;
            let py = canvas.height - (p.y / 100 * canvas.height);
            if (Math.hypot(mx - px, my - py) < 8) {
                selectedPoint = p;
                draggingPoint = p;
                updatePanel();
                updateTable();
                draw();
                return;
            }
        }

        selectedPoint = null;
        updatePanel();
        updateTable();
        draw();
    });

    canvas.addEventListener("mousemove", e => {
        if (!draggingPoint) return;

        const rect = canvas.getBoundingClientRect();
        let mx = e.clientX - rect.left;
        let my = e.clientY - rect.top;

        draggingPoint.x = Math.max(0, Math.min(100, mx / canvas.width * 100));
        draggingPoint.y = Math.max(0, Math.min(100, (canvas.height - my) / canvas.height * 100));

        sortPoints();
        updatePanel();
        updateTable();
        draw();
    });

    canvas.addEventListener("mouseup", () => draggingPoint = null);
    canvas.addEventListener("mouseleave", () => draggingPoint = null);

    function updatePanel() {
        if (!selectedPoint) {
            document.getElementById("pointInfo").innerText = "None";
            document.getElementById("xInput").value = "";
            document.getElementById("yInput").value = "";
            return;
        }

        document.getElementById("pointInfo").innerText =
            `X: ${selectedPoint.x.toFixed(2)} | Y: ${selectedPoint.y.toFixed(2)}`;
        document.getElementById("xInput").value = selectedPoint.x.toFixed(2);
        document.getElementById("yInput").value = selectedPoint.y.toFixed(2);
    }

    function updatePoint() {
        if (!selectedPoint) return;

        let x = parseFloat(document.getElementById("xInput").value);
        let y = parseFloat(document.getElementById("yInput").value);
        if (isNaN(x) || isNaN(y)) return;

        selectedPoint.x = Math.max(0, Math.min(100, x));
        selectedPoint.y = Math.max(0, Math.min(100, y));

        sortPoints();
        updatePanel();
        updateTable();
        draw();
    }

    function deletePoint() {
        if (!selectedPoint) return;
        if (points.length <= 2) return;
        if (selectedPoint === points[0] || selectedPoint === points[points.length - 1]) return;

        points = points.filter(p => p !== selectedPoint);
        selectedPoint = null;
        updatePanel();
        updateTable();
        draw();
    }

    function sanitizeFilename(name) {
        return name.replace(/[^a-z0-9_\-]/gi,"");
    }

    function exportJSON() {
        let userInput = document.getElementById("filenameInput").value.trim();
        if (userInput === "") userInput = "Default";

        userInput = sanitizeFilename(userInput);
        let finalName = "AxisConfig_" + userInput + ".json";

        const data = { axisProfile: points };
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
        const url = URL.createObjectURL(blob);

        const a = document.createElement("a");
        a.href = url;
        a.download = finalName;
        a.click();

        URL.revokeObjectURL(url);
    }

    async function exportJSONWithSave() {
        let userInput = document.getElementById("filenameInput").value.trim();
        if (userInput === "") userInput = "Default";
        userInput = "AxisConfig_" + sanitizeFilename(userInput) + ".json";

        // Check if browser support File System Access API
        if (!window.showSaveFilePicker) {
            alert("browser does not support FS Access API");
            exportJSON();
            return;
        }

        try {
            const options = {
                suggestedName: userInput,
                types: [{
                    description: "JSON File",
                    accept: {"application/json": [".json"]}
                }]
            };

            const handle = await window.showSaveFilePicker(options);
            const writable = await handle.createWritable();
            const data = { axisProfile: points };
            await writable.write(JSON.stringify(data, null, 2));
            await writable.close();
            alert("File saved");
        } catch(err) {
            console.error("Failed to save file!", err);
        }
    }

    function loadJSON(event) {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = function(e) {
            try {
                const data = JSON.parse(e.target.result);
                if (data.axisProfile && Array.isArray(data.axisProfile)) {
                    points = data.axisProfile;
                    sortPoints();
                    selectedPoint = null;
                    updatePanel();
                    updateTable();
                    draw();
                }
            } catch(err) {
                alert("Invalid JSON");
            }
        };
        reader.readAsText(file);
    }

    // 暴露给 HTML 按钮调用
    window.addPoint = addPoint;
    window.updatePoint = updatePoint;
    window.deletePoint = deletePoint;
    window.exportJSON = exportJSON;
    window.exportJSONWithSave = exportJSONWithSave;
    window.loadJSON = loadJSON;

    // 初始化
    updateTable();
    draw();
})();
