// headblock (fork feature, Experimental): the body's eye noted in the stereo pre-callback.
// Textual fragment, included by Plugin.cpp inside the stereo pre-callback, after the view position publish. Moved verbatim; not compiled on its own.
            // The body's eye for the head block, at full precision (LWC world coordinates).
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                halo::headblock_note_pre(index, p->x, p->y, p->z);
            } else {
                halo::headblock_note_pre(index, position->x, position->y, position->z);
            }
