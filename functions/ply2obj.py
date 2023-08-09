import trimesh

def convert_ply_to_obj(input_path, output_path):
    # Charge le fichier PLY
    mesh = trimesh.load(input_path)

    # Enregistre le mesh au format OBJ
    mesh.export(output_path, file_type='obj')

def main():
    ply_file_path = "FleurDeLisThing.ply"
    obj_file_path = "fichier.obj"

    convert_ply_to_obj(ply_file_path, obj_file_path)

if __name__ == "__main__":
    main()
