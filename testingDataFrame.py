    print("--- DEBUG: Inspecting full_path contents ---")
    for i, p in enumerate(full_path):
        print(f"Path object {i}: type={type(p)}")
        if hasattr(p, '__dict__'):
            print(f"  __dict__: {vars(p)}")
        else:
            print(f"  value: {p}")
        if hasattr(p, 'param'):
            print(f"  p.param: {p.param} (type: {type(p.param)})")
        if hasattr(p, 'steering'):
            print(f"  p.steering: {p.steering}")
        if hasattr(p, 'gear'):
            print(f"  p.gear: {p.gear}")
    print("--- END DEBUG ---")