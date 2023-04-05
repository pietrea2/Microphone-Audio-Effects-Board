#define _CRT_SECURE_NO_WARNINGS

#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>

#define BYTES_PER_LINE  25

/**
 * Read file in binary mode
 * @param const std::string&            path
 * @param std::vector<uint8_t>&         out_buffer
 * @return  bool                        True if the file was read, false otherwise
 */
bool read_file_binary(const std::string& path, std::vector<uint8_t>& out_buffer)
{
    std::ifstream file_stream(path, std::fstream::binary);
    if (!file_stream)
        return false;

    file_stream.seekg(0, file_stream.end);
    auto length = file_stream.tellg();
    file_stream.seekg(0, file_stream.beg);

    out_buffer.resize(length);
    file_stream.read((char*)out_buffer.data(), length);

    file_stream.close();
    return true;
}

/**
 * Write file in text mode
 * @param   const std::string&        path
 * @param   std::string&              in_buffer
 * @return  bool                      True if the file was written, false otherwise
 */
bool write_file_text(const std::string& path, const std::string& in_buffer)
{
    std::ofstream file_stream(path, std::ofstream::out);
    if (!file_stream)
        return false;

    file_stream << in_buffer;
    file_stream.close();
    return true;
}

/**
 * Generate the header with the given buffer
 * @param   std::vector<uint8_t>&   in_buffer       Input buffer
 * @param   int                     bytes_per_line  Number of bytes per line
 * @return  std::string                             Header file
 */
std::string generate_header(std::vector<uint16_t>& in_buffer, int bytes_per_line)
{
    std::string header;
    header.append("#pragma once\n");
    header.append("#include <cstdint>\n\n");
    header.append("const uint16_t image[] = \n");
    header.append("{");

    int counter = 0;
    for (auto current_char : in_buffer)
    {
        if (counter % 25 == 0)
            header.append("\n    ");

        char hex_string[64];
        sprintf_s(hex_string, "0x%04hX", current_char);

        header.append(hex_string);

        if (counter != in_buffer.size() - 1)
            header.append(", ");

        counter++;
    }

    header.append("\n};\n");
    return header;
}

/**
 * Entry point
 */
int main(int argc, char** argv)
{



    // Ensure parameters are correctly entered
    if (argc <= 2)
    {
        // Print an example
        std::cout << "Incorrect usage!" << std::endl;
        std::cout << "Syntax: " << argv[0] << " <input_binary.exe> <output_header.h> <bytes_per_line>" << std::endl;
        std::cout << "<input_binary.exe>    Path to any binary file" << std::endl;
        std::cout << "<output_header.h>     Path to output file" << std::endl;
        std::cout << "<bytes_per_line>      Optionnal: Number of bytes for each line (default: " << BYTES_PER_LINE << ")" << std::endl;
        std::cout << "Example: " << argv[0] << " image.exe image.h" << std::endl;
        return -1;
    }

    // Read first parameter
    char input_path[2048];
    strcpy_s(input_path, 2048, argv[1]);

    // Read second parameter
    char output_path[2048];
    strcpy_s(output_path, 2048, argv[2]);

    // Print paths
    //std::cout << INFO << "Input file: " << input_path << std::endl;
    //std::cout << INFO << "Output file: " << output_path << std::endl;

    std::vector<uint16_t> data;

    unsigned char bytes[2];
    FILE* fp = fopen(input_path, "rb");
    while (fread(bytes, 2, 1, fp) != 0) {
        data.push_back((((uint16_t)bytes[1] << 8) | bytes[0]));
    }

    write_file_text(output_path, generate_header(data, BYTES_PER_LINE));

    //// 
    //int bytes_per_line = BYTES_PER_LINE;
    //if (argc == 4)
    //{
    //    bytes_per_line = strtol(argv[3], NULL, 10);
    //    std::cout << INFO << "Using " << bytes_per_line << " bytes per line" << std::endl;;
    //}
    //
    //// Read input file
    //std::vector<uint8_t> buffer;
    //if (!read_file_binary(input_path, buffer))
    //{
    //    std::cout << ERROR << "Couldn't read input file" << std::endl;
    //    return -1;
    //}
    //
    //// Print some info
    //std::cout << INFO << "Input file size: 0x" << std::hex << std::uppercase << buffer.size() << std::endl;
    //std::cout << INFO << "Generating file, please wait..." << std::endl;
    //
    //// Generate header file
    //std::string header = generate_header(buffer, bytes_per_line);
    //
    //// Write file
    //if (!write_file_text(output_path, header))
    //{
    //    std::cout << ERROR << "Couldn't write output file" << std::endl;
    //    return -1;
    //}
    //
    //// Success!
    //std::cout << SUCCESS << "Success!" << std::endl;
    //return 0;
}
