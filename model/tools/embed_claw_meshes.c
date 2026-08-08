/* SPDX-License-Identifier: Apache-2.0 */
/* One-shot generator for the pinned OpenArm v1.0 binary collision STLs.
 *
 * The generated include stores every source vertex as a double literal. The
 * upstream STL coordinates are IEEE-754 binary32 millimetres; promoting them
 * here preserves their exact values while all runtime transforms and collision
 * calculations remain binary64.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t little_u32(const unsigned char bytes[4]) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8u) |
           ((uint32_t)bytes[2] << 16u) | ((uint32_t)bytes[3] << 24u);
}

static int write_mesh(FILE *output, const char *path, const char *name,
                      uint32_t expected_triangles) {
    unsigned char header[84];
    FILE *input = fopen(path, "rb");
    uint32_t triangle_count;
    uint32_t triangle;
    if (input == NULL || fread(header, 1u, sizeof(header), input) != sizeof(header)) {
        if (input != NULL) fclose(input);
        return 0;
    }
    triangle_count = little_u32(&header[80]);
    if (triangle_count != expected_triangles) {
        fclose(input);
        return 0;
    }
    fprintf(output, "static constexpr double %s[%uu][3u][3u] = {\n", name,
            triangle_count);
    for (triangle = 0u; triangle < triangle_count; ++triangle) {
        unsigned char record[50];
        uint32_t vertex;
        if (fread(record, 1u, sizeof(record), input) != sizeof(record)) {
            fclose(input);
            return 0;
        }
        fputs("  {", output);
        for (vertex = 0u; vertex < 3u; ++vertex) {
            uint32_t axis;
            fputc('{', output);
            for (axis = 0u; axis < 3u; ++axis) {
                const unsigned char *bytes = &record[12u + vertex * 12u + axis * 4u];
                const uint32_t bits = little_u32(bytes);
                float source;
                memcpy(&source, &bits, sizeof(source));
                fprintf(output, "%s%.17g", axis == 0u ? "" : ", ",
                        (double)source);
            }
            fputs(vertex == 2u ? "}" : "}, ", output);
        }
        fputs(triangle + 1u == triangle_count ? "}\n" : "},\n", output);
    }
    fputs("};\n", output);
    if (fgetc(input) != EOF) {
        fclose(input);
        return 0;
    }
    fclose(input);
    return 1;
}

int main(int argc, char **argv) {
    FILE *output;
    if (argc != 4) {
        fprintf(stderr, "usage: %s HAND_STL FINGER_STL OUTPUT_INC\n", argv[0]);
        return EXIT_FAILURE;
    }
    output = fopen(argv[3], "wb");
    if (output == NULL) return EXIT_FAILURE;
    fputs("/* SPDX-License-Identifier: Apache-2.0 */\n"
          "/* Generated from enactic/openarm_description commit\n"
          " * 6c7b720f1ba48e8bafa3a3dc752c45f397b42221.\n"
          " * hand.stl SHA-256:\n"
          " * 8e5d373ebbd3fd001b506058644062ad71a68f1ced5ca5d5ed0f6de20137956b\n"
          " * finger.stl SHA-256:\n"
          " * 8e96e1314618cf434908f70df78f68dd2b049c03538964e8d41fc99abe41564d\n"
          " * Do not edit by hand; regenerate with embed_claw_meshes.c. */\n",
          output);
    if (!write_mesh(output, argv[1], "kHandTrianglesMm", 364u) ||
        !write_mesh(output, argv[2], "kFingerTrianglesMm", 264u)) {
        (void)fclose(output);
        return EXIT_FAILURE;
    }
    if (fclose(output) != 0) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
