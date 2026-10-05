# Security and privacy reporting

Use this repository's **Security → Report a vulnerability** private reporting route for credentials, private-data exposure or safety-control vulnerabilities. Do not include secrets or private interiors in a public issue. If private reporting is unavailable, do not publish sensitive details; request a private contact through a minimal public issue without exploit or personal data.

This project is experimental. Offline tests do not certify real-flight safety. Do not test reports on a powered/armed drone. Use the simulator or mocked transports. Review browser-origin storage, cloud image sharing, loopback server permissions and recordings before enabling hardware or Claude. If a key has been exposed, its owner should rotate it; removing it from a new export does not revoke it or erase past copies.

## Known radio-engagement hazard

An offline mock of the existing mixer script showed that enabling the AI switch after a command link has gone stale can apply a previously stored axis command before a fresh link is established. Do not enable the AI switch without a fresh, verified command link. Check engagement and re-engagement props-off; do not rely on the disconnect failsafe as a certification of safety. The public export preserves the original control code rather than silently changing flight behavior. This edge needs a separately reviewed control-code fix before relying on autonomous flight.
