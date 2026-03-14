---
name: Bug Report
about: Report a reproducible problem with firmware behaviour, GRBL protocol compliance, or motion accuracy
title: "[BUG] "
labels: bug
assignees: ''
---

## Firmware Version

Paste the output of `$I` here:

```
[VER: ...]
[OPT: ...]
```

Commit hash (if built from source):

## G-Code Sender

- Software name and version (e.g. UGS 2.0.17, Candle 1.1.7, bCNC 0.9.14, raw serial):
- Operating system:
- Connection: USB serial / Bluetooth / other

## Steps to Reproduce

1.
2.
3.

Minimal G-code reproducer (if applicable):

```gcode

```

## Expected Behaviour

What should the machine do?

## Actual Behaviour

What does the machine do instead?

## Settings Dump

Paste the full output of `$$`:

<details>
<summary>$$ output</summary>

```

```

</details>

## Status Report

Paste a `?` status report captured at or near the point of failure:

```

```

## Serial Log

Paste the relevant portion of the serial console output. Include at least 10 lines before and
after the failure:

<details>
<summary>Serial log</summary>

```

```

</details>

## Additional Context

- Is the issue reproducible on every run, or intermittent?
- Does it happen with specific G-code files only, or with any motion?
- Any recent changes to settings (`$n=value`) before the failure?
- Motor driver type (TMC5160 / DRV8825 / A4988 / TMC2208):
- Microstepping setting:
