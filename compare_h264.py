def compare_bin_files(file1_path, file2_path):
    """
    Load two .bin files and return their differences.
    
    Args:
        file1_path: Path to the first .bin file
        file2_path: Path to the second .bin file
    
    Returns:
        dict: Contains 'differences' (list of indices with differences),
              'file1_size', 'file2_size', and 'size_match' (bool)
    """
    with open(file1_path, 'rb') as f1:
        data1 = f1.read()
    
    with open(file2_path, 'rb') as f2:
        data2 = f2.read()
    
    differences = []
    min_len = min(len(data1), len(data2))
    
    # Find byte-level differences
    for i in range(min_len):
        if data1[i] != data2[i]:
            differences.append({
                'index': i,
                'file1_byte': data1[i],
                'file2_byte': data2[i]
            })
    
    return {
        'differences': differences,
        'file1_size': len(data1),
        'file2_size': len(data2),
        'size_match': len(data1) == len(data2),
        'num_differences': len(differences)
    }


if __name__ == "__main__":
    result = compare_bin_files('/home/alex/dump.bin', '/home/alex/dump_.bin')
    print(f"Differences found: {result['num_differences']}")
    print(f"File sizes match: {result['size_match']}")

    print("First 10 differences:")
    for diff in result['differences'][:10]:
        print(f"Index: {diff['index']}, File1 Byte: {diff['file1_byte']:02x}, File2 Byte: {diff['file2_byte']:02x}")