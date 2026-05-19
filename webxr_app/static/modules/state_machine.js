// Client-side mirror of the server's phase. Driven by 'phase' messages
// from the server; relays prompt + safety updates to the Overlay.
//
// Phases:
//   'idle'         pre-calibration
//   'finger_cal'   prompts driving calibration steps
//   'ready'        calibration done; trigger-to-engage hint
//   'tracking'     left-trigger held; sending wrist poses
//   'fault'        safety paused; ack required
//
// The client never decides phase transitions on its own -- it just
// reflects what the server says.

export class StateMachine {
  constructor(overlay) {
    this._overlay = overlay;
    this.phase = 'idle';
  }

  setPhase(name) { this.phase = name; }

  applyPrompt(msg) {
    if (!msg) return;
    this._overlay.setPrompt(msg.text, msg.severity || 'info');
  }

  applySafety(msg) {
    if (!msg) return;
    this._overlay.setWarning(msg.message, msg.severity || 'warn');
  }
}
