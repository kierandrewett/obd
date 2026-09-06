# Multiple adapter support

- [x] Separate diagnostic requests from ELM adapter commands.
- [x] Add native TCP connections for ELM-compatible network adapters.
- [x] Add J2534 04.04 driver discovery and ISO 15765 OBD connections.
- [x] Expose connection selection in the desktop application.
- [x] Test shared operations through simulated serial/TCP and J2534 interfaces.
- [x] Document supported connections and hardware validation limits.

Later vehicle coverage work:

- [ ] Add addressed module profiles and enhanced diagnostics for Ford, PSA, GM Opel and FCA.
- [ ] Validate manual and automatic bus switching with physical adapters.
- [ ] Integrate legacy PSA VCI libraries after establishing their API and redistribution requirements.
- [ ] Add J2534 legacy protocols, pin selection, CAN FD and DoIP where supported by drivers.
- [ ] Validate real adapters and vehicles; simulation does not establish vehicle coverage.
