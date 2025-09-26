#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openssl/md5.h>

#define SIZE 1024
#define TOTAL_FILL_BYTE 0x4000 // 往编译出的固件头填充满一个扇区，保证首次上电参数不为零。
// 计算文件的 MD5 值
void compute_md5(char *file_path, unsigned char *result)
{
    int i;
    FILE *inFile = fopen(file_path, "rb");
    MD5_CTX mdContext;
    int bytes;
    unsigned char data[SIZE];

    if (inFile == NULL)
    {
        printf("%s can't be opened.\n", file_path);
        return;
    }

    MD5_Init(&mdContext);
    while ((bytes = fread(data, 1, SIZE, inFile)) != 0)
        MD5_Update(&mdContext, data, bytes);
    MD5_Final(result, &mdContext);

    fclose(inFile);
}

int main(int argc, char *argv[])
{
    // 检查命令行参数的数量
    if (argc != 15)
    {
        printf("Usage: %s <input file> <output file> <magic byte 1> <magic byte 2> <magic byte 3> <magic byte 4> <extra byte 1> <extra byte 2> <extra byte 3> <extra byte 4> <extra byte 5> <extra byte 6> <extra byte 7> <extra byte 8>\n", argv[0]);
        return 1;
    }

    // 获取输入文件名和 magic 值
    char *input_file = argv[1];
    char *output_file = argv[2];
    unsigned char magic[4];
    unsigned char extra[8];
    for (int i = 0; i < 4; i++)
    {
        char *end;
        long value = strtol(argv[i + 3], &end, 10);
        if (*end != '\0' || value < 0 || value > 255)
        {
            printf("Error: Invalid magic byte value.\n");
            return 1;
        }
        magic[i] = (unsigned char)value;
    }
    for (int i = 0; i < 8; i++)
    {
        char *end;
        long value = strtol(argv[i + 7], &end, 10);
        if (*end != '\0' || value < 0 || value > 255)
        {
            printf("Error: Invalid extra byte value.\n");
            return 1;
        }
        extra[i] = (unsigned char)value;
    }

    // 打开输入文件
    FILE *fp = fopen(input_file, "rb");
    if (fp == NULL)
    {
        printf("Failed to open file");
        return 1;
    }

    // 计算输入文件的长度
    fseek(fp, 0, SEEK_END);
    unsigned int len = ftell(fp);
    rewind(fp);

    // 计算输入文件的 MD5 值
    unsigned char md5[MD5_DIGEST_LENGTH];
    compute_md5(input_file, md5);

    // 创建输出文件
    FILE *out_fp = fopen(output_file, "wb");
    if (out_fp == NULL)
    {
        printf("Failed to open output file");
        return 1;
    }

    // 在输出文件的头部写入 magic 值、长度信息和 MD5 值
    fwrite(&magic, sizeof(magic), 1, out_fp);
    fwrite(&extra, sizeof(extra), 1, out_fp);
    fwrite(&len, sizeof(len), 1, out_fp);
    fwrite(md5, sizeof(md5), 1, out_fp);
#if TOTAL_FILL_BYTE > 0
    // 填充满一个扇区
    unsigned char fill[TOTAL_FILL_BYTE];
    memset(fill, 0xff, sizeof(fill));
    fwrite(fill, (sizeof(fill) - ftell(out_fp)), 1, out_fp);
#endif
    // 将输入文件的内容复制到输出文件中
    char buffer[SIZE];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), fp)) > 0)
    {
        fwrite(buffer, 1, bytes, out_fp);
    }

    // 关闭文件
    fclose(fp);
    fclose(out_fp);

    return 0;
}