MIT License

Copyright (c) 2026 NitsuJack

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

This licence covers **this project's own** code, configuration and original art.
It does not cover, and this project does not redistribute, any Halo game asset.

The release also bundles `plugins\CutsceneDetectionPlugin.dll` (cutscene comfort), which is
**elliotttate's** work (https://github.com/elliotttate), not ours, and so is not covered by the MIT
licence above. It is a standalone UEVR plugin that UEVR loads separately -- it is not linked into
this project's code, and neither is a derivative of the other.

The UEVR plugin SDK headers this project builds against are © praydog and are
**not** vendored here — they are fetched from https://github.com/praydog/UEVR at
build time under their own terms. See COMPILING.md.
