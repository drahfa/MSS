// Telemetry Logging, Export (CSV/JSON), and Deterministic Flight Replay System.

export class TelemetryRecorder {
  constructor() {
    this.frames = [];
    this.isRecording = false;
    this.isReplaying = false;
    this.replayIndex = 0;
  }

  startRecording() {
    this.frames = [];
    this.isRecording = true;
    this.isReplaying = false;
    this.replayIndex = 0;
  }

  stopRecording() {
    this.isRecording = false;
    return this.frames.length;
  }

  recordFrame(time, state, commands, pilotInput, assist) {
    if (!this.isRecording) return;
    this.frames.push({
      time: Number(time.toFixed(3)),
      state: state.map(v => Number(v.toFixed(5))),
      commands: commands.map(v => Number(v.toFixed(4))),
      pilotInput: { ...pilotInput },
      assist: Boolean(assist),
    });
  }

  exportJson() {
    return JSON.stringify({
      schema: 'mss-telemetry-v1',
      recordedAt: new Date().toISOString(),
      frameCount: this.frames.length,
      frames: this.frames,
    }, null, 2);
  }

  exportCsv() {
    if (this.frames.length === 0) return '';
    const headers = [
      'time',
      'u', 'v', 'w', 'p', 'q', 'r',
      'pos_N', 'pos_E', 'pos_D',
      'q_eta', 'q_eps1', 'q_eps2', 'q_eps3',
      'surge_cmd', 'sway_cmd', 'heave_cmd', 'yaw_cmd',
      'assist',
    ];
    const rows = this.frames.map(f => [
      f.time,
      ...f.state,
      f.pilotInput?.surge ?? 0,
      f.pilotInput?.sway ?? 0,
      f.pilotInput?.heave ?? 0,
      f.pilotInput?.yaw ?? 0,
      f.assist ? 1 : 0,
    ].join(','));
    return [headers.join(','), ...rows].join('\n');
  }

  loadFrames(frames) {
    if (!Array.isArray(frames) || frames.length === 0) {
      throw new Error('Invalid telemetry frames format');
    }
    this.frames = frames;
    this.isRecording = false;
    this.isReplaying = false;
    this.replayIndex = 0;
  }

  startReplay() {
    if (this.frames.length === 0) return false;
    this.isReplaying = true;
    this.isRecording = false;
    this.replayIndex = 0;
    return true;
  }

  stopReplay() {
    this.isReplaying = false;
    this.replayIndex = 0;
  }

  getNextReplayFrame() {
    if (!this.isReplaying || this.replayIndex >= this.frames.length) {
      this.isReplaying = false;
      return null;
    }
    const frame = this.frames[this.replayIndex];
    this.replayIndex++;
    return frame;
  }

  getReplayProgress() {
    if (this.frames.length === 0) return 0;
    return this.replayIndex / this.frames.length;
  }
}
