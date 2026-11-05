"""Bounded timing samples; display work never queues behind old frames."""
class StageTimings:
    def __init__(self, capacity=120):
        self.capacity = capacity
        self.lock = threading.Lock()
        self.samples = {}

    def add(self, name, milliseconds):
        if not math.isfinite(milliseconds) or milliseconds < 0:
            return
        with self.lock:
            self.samples.setdefault(name, deque(maxlen=self.capacity)).append(milliseconds)

    def snapshot(self):
        with self.lock:
            copied = {name: list(values) for name, values in self.samples.items()}
        return {name: dict(samples=len(values), last_ms=round(values[-1], 2),
                           p50_ms=round(sorted(values)[math.ceil(.5*len(values))-1], 2),
                           p95_ms=round(sorted(values)[math.ceil(.95*len(values))-1], 2))
                for name, values in copied.items() if values}


class LatestPreview:
    """One in-flight render plus one replaceable pending frame, never a FIFO.

    The renderer owns its image until finished. Local clicks use only the frame
    which successfully reached Display.show, not the newer inference mailbox.
    """
    def __init__(self, render, fps, timings, clock=time.monotonic):
        self.render, self.period, self.timings, self.clock = render, 1/fps, timings, clock
        self.condition = threading.Condition()
        self.pending = self.displayed = None
        self.stopping = False
        self.error = None
        self.replaced = self.rendered = 0
        self.completed = deque(maxlen=60)
        self.thread = threading.Thread(target=self._run, name="local-preview", daemon=True)

    def offer(self, frame, image, selected, prediction):
        with self.condition:
            if self.stopping:
                return
            if self.pending is not None:
                self.replaced += 1
            self.pending = (frame, image, selected, prediction)
            self.condition.notify()

    def take(self):
        with self.condition:
            job, self.pending = self.pending, None
            return job

    def _run(self):
        deadline = 0.
        previous = None
        try:
            while True:
                with self.condition:
                    while not self.stopping:
                        wait = deadline-self.clock()
                        if (self.pending is not None or previous is not None) and wait <= 0:
                            break
                        self.condition.wait(max(.001, wait) if self.pending is not None or previous is not None else None)
                    if self.stopping:
                        return
                    fresh = self.pending is not None
                    job, self.pending = self.pending or previous, None
                started = self.clock()
                self.timings.add("capture_to_preview_start", max(0., started-job[0]["stamp"])*1000)
                self.render(*job)
                finished = self.clock()
                with self.condition:
                    self.displayed = job[0]
                    if fresh:
                        self.rendered += 1
                        self.completed.append(finished)
                self.timings.add("local_preview", (finished-started)*1000)
                self.timings.add("capture_to_display_submit", max(0., finished-job[0]["stamp"])*1000)
                # Start-to-start cadence, rather than adding another period to rendering time.
                previous = job
                deadline = started+(self.period if fresh else max(.2, self.period))
        except Exception as exc:
            self.error = exc

    def stats(self):
        with self.condition:
            completed = list(self.completed)
        fps = ((len(completed)-1)/(completed[-1]-completed[0])
               if len(completed)>1 and completed[-1]>completed[0] else 0.)
        return dict(local_preview_fps_actual=round(fps, 1), preview_replaced_frames=self.replaced,
                    preview_rendered_frames=self.rendered)

    def close(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        if self.thread.ident is not None:
            self.thread.join(2.)
            if self.thread.is_alive():
                raise RuntimeError("Display thread did not stop; camera resources retained")
