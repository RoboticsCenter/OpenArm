#!/usr/bin/env bash
# Generate a TLS certificate chain for the motor dashboard into certs/.
#
# Adapted from the Camera-SensorPortal (Spyglass) approach. Produces a proper
# two-cert setup (NOT a single self-signed cert that doubles as its own CA —
# Firefox rejects that with MOZILLA_PKIX_ERROR_CA_CERT_USED_AS_END_ENTITY):
#
#   ca.crt / ca.key          a local root CA. Clients IMPORT ca.crt as trusted.
#   server.crt / server.key  a leaf cert signed by that CA, presented by Flask.
#                            Covers localhost, this host's name, and its LAN IPs (SAN).
#
# Because the CA is stable, regenerating only the leaf (e.g. after the host's IP
# changes) does NOT require re-importing on clients — they already trust the CA.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)/certs"
mkdir -p "$DIR"
CA_CRT="$DIR/ca.crt"
CA_KEY="$DIR/ca.key"
CRT="$DIR/server.crt"
KEY="$DIR/server.key"
CSR="$DIR/server.csr"

host="$(hostname)"
san="DNS:localhost,DNS:${host},DNS:${host}.local,IP:127.0.0.1"
for ip in $(hostname -I 2>/dev/null); do
    case "$ip" in *:*) ;; *) san="${san},IP:${ip}" ;; esac   # skip IPv6 for simplicity
done

echo "Generating TLS certificate chain"
echo "  subjectAltName = ${san}"

# 1. Root CA (self-signed). This is the file clients install as trusted. It is a
#    CA only (CA:TRUE, keyCertSign) and is NEVER presented as the server cert, so
#    Firefox/Chrome/Apple all accept it. Longer-lived so it's a one-time install;
#    Apple's 825-day limit applies to the *leaf* (server) cert, not the root.
if [ ! -f "$CA_CRT" ] || [ ! -f "$CA_KEY" ]; then
    openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
        -keyout "$CA_KEY" -out "$CA_CRT" -days 3650 \
        -subj "/CN=Motor Dashboard Local CA/O=MotorDashboard" \
        -addext "basicConstraints=critical,CA:TRUE" \
        -addext "keyUsage=critical,keyCertSign,cRLSign" 2>/dev/null
    echo "  created root CA (ca.crt) — install THIS on client devices"
else
    echo "  reusing existing root CA (ca.crt) — no client re-import needed"
fi

# 2. Server leaf key + CSR (no -addext here; extensions are applied at signing).
openssl req -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
    -keyout "$KEY" -out "$CSR" \
    -subj "/CN=Motor Dashboard/O=MotorDashboard" 2>/dev/null

# 3. Sign the leaf with the CA. CA:FALSE + serverAuth + SAN make it a valid TLS
#    server cert; <=825 days keeps Apple platforms happy.
openssl x509 -req -in "$CSR" -CA "$CA_CRT" -CAkey "$CA_KEY" -CAcreateserial \
    -days 825 -out "$CRT" \
    -extfile <(printf 'subjectAltName=%s\nbasicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature\nextendedKeyUsage=serverAuth\n' "$san") \
    2>/dev/null

rm -f "$CSR" "$DIR/ca.srl"
chmod 600 "$KEY" "$CA_KEY"

echo "Wrote:"
echo "  $CA_CRT   <- install THIS as a trusted certificate on client devices"
echo "  $CRT  (server leaf, presented over TLS; signed by ca.crt)"
echo "  $KEY"
echo
echo "If the host's IP changes, re-run this — the CA is reused, so the leaf is"
echo "re-signed and clients do NOT need to re-import."
echo "To remove the browser warning, install $CA_CRT as a trusted certificate on"
echo "the client; otherwise click 'Advanced -> Proceed' once per device."
