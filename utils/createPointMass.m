function name = createPointMass(name)
opensimroot = 'C:\opensim 4.3\'; %create a char array that has the opensim path toplevel directory
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add the opensim path to the
javaaddpath([opensimroot 'bin'], [opensimroot 'sdk\lib']); %add opensimroot bin and the java path to MATLAB's dynamic path path
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);% Set Windows System path to include OpenSim libraries
import org.opensim.modeling.* %import opensim api library

model = Model();
model.setName('');

model.setGravity(Vec3(0, -9.80665, 0));

body1 = Body();
body1.setName("body1");
body1.setMass(1);
body1.setInertia(Inertia(0));
body1.attachGeometry(Sphere(.05));
model.addBody(body1);

% Allows translation along x.
joint = SliderJoint('slider', model.getGround(), body1);
coord = joint.updCoordinate();
coord.setName('position');
coord.setRange([0 10]);
model.addComponent(joint);

actu = CoordinateActuator();
actu.setCoordinate(coord);
actu.setName('actuator');
actu.setOptimalForce(1);
model.addComponent(actu);


model.finalizeConnections();
model.print(name);
display(name +" printed!");
end