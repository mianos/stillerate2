// Advisory PID tuner configuration. Runs on deploy (and on every restart) before
// any message arrives, so every other node can assume it exists.
flow.set('cfg', {
    // --- endpoints ---------------------------------------------------------
    stillBase: 'http://still2.mianos.com',
    llmUrl: 'http://docker-host.mianos.com:8082/v1/chat/completions',
    llmModel: 'LiquidAI/LFM2.5-24B-A2B-GGUF',

    // --- safety ------------------------------------------------------------
    dryRun: false,            // false = Apply really POSTs; set true to rehearse dry runs
    maxDeltaPct: 30,          // max % change per gain, per proposal
    minProposalGapMin: 5,     // minimum spacing between LLM calls
    revertAfterMin: 15,       // watchdog horizon after an apply
    maxSetPointStep: 0.5,     // degC per proposal
    llmTimeoutMs: 90000,

    // --- digest ------------------------------------------------------------
    windowMin: 10,            // analysis window
    csvRows: 50,              // downsampled telemetry rows sent to the model
    maxSampleAgeSec: 30,      // refuse stale telemetry (e.g. a retained message)
    maxTokens: 500,

    // --- watchdog ----------------------------------------------------------
    watchdogWorsePct: 25,     // revert if mae worsens by this much
    watchdogBiasMargin: 0.3,  // degC of extra bias tolerated
    watchdogSatPct: 40,       // revert if this share of samples is saturated

    // --- hard envelope: mirror of PIDController + README. The LLM may never
    // leave it, whatever it argues; the validator drops the whole proposal.
    limits: {
        kp: [0, 8], ki: [0, 1], kd: [0, 60],
        set_point: [70, 85], output_min: [0, 50], output_max: [20, 100],
        integral_limit: [0, 10000], wind_down_threshold: [0, 10],
        integral_unwind_factor: [0, 0.9], derivative_filter: [0, 0.9]
    }
});
if (!Array.isArray(flow.get('buf'))) flow.set('buf', []);
