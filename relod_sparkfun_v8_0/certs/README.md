# OTA trust roots

`ota-roots.pem` contains ISRG Root X1/X2 and Amazon Root CA 1–4, downloaded
from the CA operators on 2026-09-28:

- https://letsencrypt.org/certificates/
- https://www.amazontrust.com/repository/

These public trust roots authenticate both the Fly metadata connection and S3
download. Both OTA clients require certificate and hostname validation, a valid
clock, and HTTPS without redirects. Never replace this with `setInsecure()`.
Review CA expiry and server chain compatibility during every hardware pilot.
Changing roots requires a firmware release before the old chain stops working.

Measurement uploads retain their existing transport behavior; this change only
secures OTA. SHA-256 verifies downloaded bytes against the authenticated manifest.
It is not an independent firmware signature or secure-boot implementation.
