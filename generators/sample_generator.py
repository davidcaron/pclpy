from generators.main_generator import generate, write_stuff_if_needed


def main():
    headers = [
        ("stereo", "stereo_matching.h", ""),
        ("io", "file_io.h", ""),
    ]
    generated_headers = generate(headers)
    write_stuff_if_needed(generated_headers, delete_others=False)


if __name__ == '__main__':
    main()
