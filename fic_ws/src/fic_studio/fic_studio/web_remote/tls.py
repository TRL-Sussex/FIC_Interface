"""The web remote's certificate, so passwords do not cross the lab in the clear.

Each Studio makes its own self-signed certificate the first time it serves, for
its host name and every address it answers on, and keeps it for a year. A
browser will warn the first time, because nothing signed it — the lab can
install the certificate on the devices that use it to stop that.

Uses the ``cryptography`` module when it is there (it ships with Ubuntu) and
falls back to the ``openssl`` command.
"""
import datetime
import ipaddress
import os
import socket
import subprocess

#: How long a generated certificate lasts.
DAYS = 365
#: Remake it when this little is left, so it never expires mid-experiment.
RENEW_DAYS = 30


def ensure_cert(directory, addresses=()):
    """Return (cert path, key path) for this machine, making them if needed."""
    os.makedirs(directory, exist_ok=True)
    cert = os.path.join(directory, 'web-cert.pem')
    key = os.path.join(directory, 'web-key.pem')
    if _usable(cert, key):
        return cert, key
    names = _names(addresses)
    try:
        _make_with_cryptography(cert, key, names)
    except Exception:
        _make_with_openssl(cert, key, names)
    os.chmod(key, 0o600)
    return cert, key


def _names(addresses):
    host = socket.gethostname().split('.')[0]
    out = [host, 'localhost']
    for address in addresses:
        if address and address not in out:
            out.append(address)
    if '127.0.0.1' not in out:
        out.append('127.0.0.1')
    return out


def _usable(cert, key):
    """Say whether both files exist and the certificate is not about to expire."""
    if not (os.path.exists(cert) and os.path.exists(key)):
        return False
    try:
        from cryptography import x509
        with open(cert, 'rb') as handle:
            loaded = x509.load_pem_x509_certificate(handle.read())
        left = loaded.not_valid_after_utc - datetime.datetime.now(datetime.timezone.utc)
        return left.days > RENEW_DAYS
    except Exception:
        return True          # cannot check it: leave what is there alone


def _sans(names):
    from cryptography import x509
    out = []
    for name in names:
        try:
            out.append(x509.IPAddress(ipaddress.ip_address(name)))
        except ValueError:
            out.append(x509.DNSName(name))
    return out


def _make_with_cryptography(cert_path, key_path, names):
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, names[0]),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, 'FIC Control Studio'),
    ])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (x509.CertificateBuilder()
            .subject_name(subject).issuer_name(subject)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(minutes=5))
            .not_valid_after(now + datetime.timedelta(days=DAYS))
            .add_extension(x509.SubjectAlternativeName(_sans(names)), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .sign(key, hashes.SHA256()))
    with open(key_path, 'wb') as handle:
        handle.write(key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption()))
    with open(cert_path, 'wb') as handle:
        handle.write(cert.public_bytes(serialization.Encoding.PEM))


def _make_with_openssl(cert_path, key_path, names):
    alt = ','.join(('IP:%s' % n) if _is_ip(n) else ('DNS:%s' % n) for n in names)
    subprocess.run(
        ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
         '-keyout', key_path, '-out', cert_path, '-days', str(DAYS),
         '-subj', '/CN=%s/O=FIC Control Studio' % names[0],
         '-addext', 'subjectAltName=%s' % alt],
        check=True, capture_output=True, timeout=120)


def _is_ip(name):
    try:
        ipaddress.ip_address(name)
        return True
    except ValueError:
        return False


def context(cert, key):
    """Return an SSL context serving ``cert``/``key``."""
    import ssl
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(cert, key)
    return ctx
