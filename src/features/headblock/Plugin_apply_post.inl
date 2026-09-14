// headblock (fork feature, Experimental): the composed eye pulled back out of geometry in the stereo post-callback.
// Textual fragment, included by Plugin.cpp inside the stereo post-callback, before the eye publish. Moved verbatim; not compiled on its own.
        // HEAD BLOCK: pull the composed eye back out of geometry BEFORE anything below publishes
        // it, so markers and the reticule reason from the eye that is actually rendered.
        if (position != nullptr) {
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                double hx = p->x, hy = p->y, hz = p->z;
                if (halo::headblock_apply_post(index, &hx, &hy, &hz)) { p->x = hx; p->y = hy; p->z = hz; }
            } else {
                double hx = position->x, hy = position->y, hz = position->z;
                if (halo::headblock_apply_post(index, &hx, &hy, &hz)) {
                    position->x = (float)hx; position->y = (float)hy; position->z = (float)hz;
                }
            }
        }
