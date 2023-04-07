#define _CRT_SECURE_NO_WARNINGS

#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>

// based on https://github.com/patrickcjk/file2header

int main(int argc, char** argv)
{
    if (argc <= 2)
    {
        std::cout << "Converts 32-bit PCM file to C array" << std::endl;
        std::cout << "Usage: " << argv[0] << " <input_file> <output_file> <words_per_line>" << std::endl;
        std::cout << "Example: " << argv[0] << " sound.pcm sound_data.h 10" << std::endl;
        return -1;
    }

    char input_path[_MAX_PATH];
    strcpy_s(input_path, _MAX_PATH, argv[1]);

    char output_path[_MAX_PATH];
    strcpy_s(output_path, _MAX_PATH, argv[2]);

    int words_per_line = argc == 4 ? strtol(argv[3], NULL, 10) : 10;

    std::vector<int32_t> words;

    unsigned char bytes[4];
    FILE* fp = fopen(input_path, "rb");
    while (fread(bytes, 4, 1, fp) != 0) {
        words.push_back((int32_t)(
            ((uint32_t)bytes[3] << 24) |
            ((uint32_t)bytes[2] << 16) |
            ((uint32_t)bytes[1] << 8) |
            bytes[0]));
    }

    // generate c array
    std::string str;
    str.append("const int data[] = \n");
    str.append("{");
    for (int i = 0; i < words.size(); ++i)
    {
        if (i % words_per_line == 0)
            str.append("\n    ");

        char hex[64];
        sprintf_s(hex, "0x%08X", words[i]);
        str.append(hex);

        if (i != words.size() - 1)
            str.append(", ");
    }
    str.append("\n};\n");

    // write out
    std::ofstream file(output_path, std::ofstream::out);
    if (!file) return -1;
    file << str;
    file.close();

    return 0;
}
