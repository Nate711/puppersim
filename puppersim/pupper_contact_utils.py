def get_ground_contacts(env):
  ground_contacts = []
  for ground_id in env._scene.ground_ids:
    ground_contacts += env._pybullet_client.getContactPoints(bodyA=ground_id)
  return ground_contacts

def number_non_foot_contacts(env):
  ground_contacts = get_ground_contacts(env)
  links_in_contact = [contact[4] for contact in ground_contacts]
  foot_links = env.robot._urdf_loader.get_end_effector_id_dict().values()
  return len([link for link in links_in_contact if link not in foot_links])

def has_robot_crashed(env):
  return number_non_foot_contacts(env) > 0