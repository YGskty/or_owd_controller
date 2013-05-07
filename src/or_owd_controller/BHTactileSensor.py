import numpy, openravepy, types

def RenderForces(module, length=0.1, width=2, color=None):
    if color is None:
        color = numpy.array([ 1, 1, 0, 1 ], dtype='float')

    data = module.GetSensorData()
    lines = numpy.empty((2 * data.positions.shape[0], data.positions.shape[1]))
    lines[0::2] = data.positions
    lines[1::2] = data.positions + data.forces * length
    return module.GetEnv().drawlinelist(lines, width, color)

def Bind(module):
    cls = type(module)
    module.RenderForces = types.MethodType(RenderForces, module, cls) 
    return module

def BHTactileSensor(env, node_name, namespace, robot, link_prefix):
    args = [ 'BHTactileSensor', node_name, namespace, robot.GetName(), link_prefix ]
    args_str = ' '.join(args)
    module = openravepy.RaveCreateSensor(env, args_str)
    if module is None:
        raise Exception('Creating BHTactileSensor failed.')

    env.Add(module, True)
    return Bind(module)
