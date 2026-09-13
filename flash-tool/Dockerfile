# DroneBridge JONOKR build image = ESP-IDF 5.4.x + Node.js.
# The firmware build runs `npm install && npm run build` to inline the web UI into www.bin,
# but the stock espressif/idf image ships no Node.js, so we add Node 20 LTS here.
# Built automatically by build-fw.ps1:  docker build -t jonokr-idf flash-tool
#
# Pinned to the exact IDF point release the repo targets (immutable version tag) for
# reproducibility. For maximum supply-chain assurance, pin by digest instead, e.g.
#   FROM espressif/idf@sha256:<digest>
FROM espressif/idf:v5.4.4

# Node.js 20 LTS via NodeSource (the frontend's npm devDeps want Node >= 20).
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
       | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" \
       > /etc/apt/sources.list.d/nodesource.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends nodejs \
    && node --version && npm --version \
    && rm -rf /var/lib/apt/lists/*

# The base image's ENTRYPOINT sets up the IDF environment and then execs our command,
# so `docker run jonokr-idf bash -c "idf.py ..."` runs with idf.py + node on PATH.
