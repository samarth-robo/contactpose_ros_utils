from lxml import etree
import os
import yaml

osp = os.path


def create_mocap_file(assets_filename=osp.join('..', 'data', 'contactdb_data',
                              'optitrack_rigid_bodies', 'contactdb_assets.motive')):
  data = etree.parse(assets_filename).getroot()
  data = data[0]
  mocap_ids = {}
  for rigid_body in data:
    for properties in rigid_body:
      if properties.tag != 'properties':
        continue
      object_name = properties[0][1].text
      if ('LargeA2Saruco' in object_name) or ('SymmetricC' in object_name):
        print('Ignoring {:s}'.format(object_name))
        break
      for prop in properties:
        if prop[0].text != 'UserData':
          continue
        mocap_id = prop[1].text
    else:
      mocap_ids[object_name] = mocap_id

  id2object = {}
  for object_name, mocap_id in mocap_ids.items():
    if mocap_id in id2object:
      id2object[mocap_id].append(object_name)
    else:
      id2object[mocap_id] = [object_name]
  for mocap_id, object_names in id2object.items():
    if len(object_names) > 1:
      print('{:s} have the same id {:s}'.format(', '.join(object_names), mocap_id))
      return

  rigid_bodies_dict = {}
  for object_name in sorted(mocap_ids.keys()):
    print('{:s}: {:s}'.format(object_name, mocap_ids[object_name]))
    this_object = {
      'pose': '{:s}/pose'.format(object_name),
      'pose2d': '{:s}/ground_pose'.format(object_name),
      'child_frame_id': '{:s}_frame_optitrack'.format(object_name),
      'parent_frame_id': 'optitrack_frame'
    }
    rigid_bodies_dict[mocap_ids[object_name]] = this_object
  print('Total {:d} objects'.format(len(mocap_ids)))

  output_dict = {
    'rigid_bodies': rigid_bodies_dict,
    'optitrack_config': {
      'multicast_address': '239.255.42.99',
      'command_port': 1510,
      'data_port': 1511
    }
  }

  filename = osp.join('..', 'data', 'mocap.yaml')
  with open(filename, 'w') as f:
    yaml.dump(output_dict, f, default_flow_style=False)
  print('{:s} written'.format(filename))


if __name__ == '__main__':
  create_mocap_file()