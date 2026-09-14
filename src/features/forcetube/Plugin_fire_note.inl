// forcetube (fork feature, Experimental): the player's own fire input, sampled on the raw pad for the gunstock kick.
// Textual fragment, included by Plugin.cpp inside the XInput hook, after the holster button note. Moved verbatim; not compiled on its own.
        // The player's OWN fire input, sampled here because this is still the RAW pad -- our own
        // reload/holster suppression and the synthetic presses all happen further down, and the
        // haptics must follow the finger, not the composed state.
        forcetube_note_fire(state->Gamepad.bRightTrigger >= 64 ||
                            (state->Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0);
