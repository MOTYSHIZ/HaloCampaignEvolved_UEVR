// heightcal (fork feature, Experimental): the heightmode word parser.
// Textual fragment, included by Config.cpp at namespace halo scope, before parse_fork_port_key(). Moved verbatim; not compiled on its own.
// heightmode takes a word (absolute, seated, eyes) or its number.
static int parse_height_mode(const char* val, double v) {
    while (*val == ' ' || *val == '\t') ++val;
    if (_strnicmp(val, "absolute", 8) == 0) return 0;
    if (_strnicmp(val, "seated", 6) == 0)   return 1;
    if (_strnicmp(val, "eyes", 4) == 0)     return 2;
    return (int)clampf((float)v, 0.0f, 2.0f);
}
