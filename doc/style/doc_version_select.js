
// The line below will be replaced to include any additional
// auto-detected versions. See script in docs.yaml.
var allVersions = ["develop", "master", "ros2"];

function buildSelect(currentVersion) {
    if (!allVersions.includes(currentVersion)) {
        allVersions.unshift(currentVersion);
    }

    // couldn't get external CSS stylesheet to apply here for some reason
    var buf = ['<span id="versionselectorlabel">Astrobee Version:</span><select id="versionselector" style="font-size: 100%; padding: 3px 5px 3px 5px; margin-left: 5px;">'];
    for (version of allVersions) {
        buf.push('<option value="' + version + '"');
        if (version == currentVersion) {
            buf.push(' selected="selected"');
        }
        buf.push(">" + version + "</option>");
    }
    buf.push("</select>");

    return buf.join("");
}

function detectCurrentVersion() {
    const version_regex = /\/v\/(.*)\//;
    var match = version_regex.exec(window.location.pathname);
    if (match) {
        return match[1];
    } else {
        return "(unknown)";
    }
}

function onSelectorChange() {
    var selector = document.getElementById("versionselector");
    var currentVersion = detectCurrentVersion();
    var selectedVersion = selector.value;
    window.location.pathname = window.location.pathname.replace(currentVersion, selectedVersion);
}

function initVersionSelector() {
    var currentVersion = detectCurrentVersion();
    var projNumDiv = document.getElementById("projectnumber");
    projNumDiv.innerHTML = buildSelect(currentVersion);
    var selector = document.getElementById("versionselector");
    // couldn't get external CSS stylesheet to apply here for some reason
    document.getElementById('projectnumber').style = "position: relative; top: -0.3em;";

    selector.addEventListener("change", onSelectorChange);
}

initVersionSelector();
