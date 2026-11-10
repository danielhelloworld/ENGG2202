"""Fusion builder. All public dimensions below are millimetres.
Creates a NEW local assembly; never changes the source camera or gimbal.
"""
import adsk.core, adsk.fusion
import os, json, math

HERE=os.path.dirname(os.path.abspath(__file__))
OUT=os.path.join(HERE,'..','output_v2')
NEW=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN=adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT=adsk.fusion.FeatureOperations.CutFeatureOperation
RISE=3.0  # Clear the existing 11.5 mm high end tabs of the arm.

def p(x,y,z=0): return adsk.core.Point3D.create(x/10,y/10,z/10)
def val(mm): return adsk.core.ValueInput.createByReal(mm/10)
def offset(c,z,base=None):
    i=c.constructionPlanes.createInput()
    i.setByOffset(base or c.xYConstructionPlane,val(z))
    return c.constructionPlanes.add(i)

def poly_sketch(c,z,pts,name):
    s=c.sketches.add(offset(c,z)); s.name=name
    for a,b in zip(pts,pts[1:]+pts[:1]): s.sketchCurves.sketchLines.addByTwoPoints(p(*a),p(*b))
    if s.profiles.count!=1: raise RuntimeError('Not one closed profile: '+name)
    s.isVisible=False
    return s.profiles.item(0)

def ext(c,profile,h,op,name):
    i=c.features.extrudeFeatures.createInput(profile,op)
    if op==CUT:i.participantBodies=[b for b in c.bRepBodies]
    e=adsk.fusion.DistanceExtentDefinition.create(val(abs(h)))
    i.setOneSideExtent(e,adsk.fusion.ExtentDirections.PositiveExtentDirection if h>=0 else adsk.fusion.ExtentDirections.NegativeExtentDirection)
    f=c.features.extrudeFeatures.add(i); f.name=name
    return f

def rect(c,z,x1,y1,x2,y2,h,op,name):
    return ext(c,poly_sketch(c,z,[(x1,y1),(x2,y1),(x2,y2),(x1,y2)],name+' sketch'),h,op,name)

def rounded(c,z,w,l,r,h,op,name):
    # Chamfered corners keep the part printable and avoid sharp hand-contact tips.
    a=w/2;b=l/2
    pts=[(-a+r,-b),(a-r,-b),(a,-b+r),(a,b-r),(a-r,b),(-a+r,b),(-a,b-r),(-a,-b+r)]
    return ext(c,poly_sketch(c,z,pts,name+' sketch'),h,op,name)

def circle(c,z,x,y,d,h,op,name,plane=None):
    s=c.sketches.add(plane or offset(c,z));s.name=name+' sketch'
    s.sketchCurves.sketchCircles.addByCenterRadius(p(x,y),d/20)
    s.isVisible=False
    return ext(c,s.profiles.item(0),h,op,name)

def loft(c,z1,pts1,z2,pts2,op,name):
    a=poly_sketch(c,z1,pts1,name+' lower');b=poly_sketch(c,z2,pts2,name+' upper')
    i=c.features.loftFeatures.createInput(op);i.loftSections.add(a);i.loftSections.add(b)
    i.isSolid=True
    f=c.features.loftFeatures.add(i);f.name=name
    return f

def component(root,name):
    for old in root.occurrences:old.isLightBulbOn=False
    o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name
    return o

def body(c):
    if c.bRepBodies.count!=1 or not c.bRepBodies.item(0).isSolid:
        raise RuntimeError(c.name+' must be exactly one solid')
    return c.bRepBodies.item(0)

def rotate_body(c):
    entities=adsk.core.ObjectCollection.create();entities.add(body(c))
    m=adsk.core.Matrix3D.create()
    m.setToRotation(math.pi/2,adsk.core.Vector3D.create(0,0,1),p(0,0,0))
    i=c.features.moveFeatures.createInput2(entities);i.defineAsFreeMove(m)
    f=c.features.moveFeatures.add(i);f.name='Align camera long edge with gimbal arm - 90 degrees'

def appearance(app,c,rgb):
    try:
        lib=app.materialLibraries.itemByName('Fusion 360 Appearance Library')
        a=lib.appearances.itemByName('Plastic - Matte (Blue)') if lib else None
        if not a:a=c.bRepBodies.item(0).appearance
        if a:
            d=adsk.fusion.Design.cast(app.activeProduct)
            a=d.appearances.addByCopy(a,c.name+' color')
            color=a.appearanceProperties.itemById('generic_diffuse') or a.appearanceProperties.itemById('metal_f0')
            if color:color.value=adsk.core.Color.create(*rgb,255)
            for b in c.bRepBodies:b.appearance=a
    except:pass

def receiver(root,name,heat=True,interference=0.10):
    o=component(root,name);c=o.component
    rounded(c,0,56,44,2,4+RISE,NEW,'56 x 44 x 7 platform')
    for sign in [-1,1]:
        def q(inner,outer,ya=-22,yb=22):
            xs=sorted([sign*inner,sign*outer])
            return [(xs[0],ya),(xs[1],ya),(xs[1],yb),(xs[0],yb)]
        # Male x = +/- (20 - (z-4.3)*0.8). Female rigid faces gap 0.25 mm.
        loft(c,4+RISE,q(20.49,28),9.2+RISE,q(16.33,28),JOIN,'Dovetail retaining rail 0.8 to 1 '+str(sign))
        # Inner rail is a longitudinal flexure, attached only at the +Y end.
        xs=sorted([sign*22.5,sign*23.4])
        rect(c,3.7+RISE,xs[0],-22,xs[1],13,5.8,CUT,'0.9 mm longitudinal flexure slot '+str(sign))
        xs=sorted([sign*15.0,sign*22.5])
        rect(c,3.7+RISE,xs[0],-22,xs[1],13,0.8,CUT,'0.8 mm flexure underside relief '+str(sign))
        def rib(z):
            edge=20.25-(z-4.3)*0.8
            # Ramp from no contact to the chosen interference over 8 mm.
            vals=[(edge+0.20,-6),(edge+0.20,11),(edge-0.25-interference,11),(edge-0.25-interference,2),(edge+0.10,-6)]
            return [(sign*x,y) for x,y in vals]
        loft(c,4.6+RISE,rib(4.6),8.8+RISE,rib(8.8),JOIN,'Tapered friction contact rib '+str(sign))
    rect(c,4+RISE,-16.5,19.4,16.5,22,5.2,JOIN,'Closed insertion end stop')
    # Open centre provides driver access and avoids contacting the screw head.
    circle(c,0,0,0,15.0,4+RISE,CUT,'15 mm screw and driver access')
    rotate_body(c)
    rect(c,0,7.8,-28,11.4,-21.95,9.3+RISE,CUT,'Clear original negative Y raised end tab and root fillet')
    rect(c,0,7.8,25.95,11.4,28,9.3+RISE,CUT,'Clear original positive Y raised end tab and root fillet')
    # The original arm is 14 mm wide, asymmetrical about its slot (-6 to +8).
    for x1,x2 in [(-8.5,-6.3),(8.3,10.5)]:
        rect(c,-2.1,x1,-20,x2,20,2.1,JOIN,'Arm lateral locating shoulder')
    for y in [-16,16]:
        if heat:
            circle(c,-4.5,0,y,2.1,4.5,JOIN,'Heat stake pin at y '+str(y))
        else:
            circle(c,0,0,y,2.4,4+RISE,CUT,'M2 through clearance at y '+str(y))
            circle(c,2.3+RISE,0,y,4.5,1.7,CUT,'M2 head recess at y '+str(y))
    body(c).name=name
    return o

def plate(root,name,frame=False):
    o=component(root,name);c=o.component
    rounded(c,9.3+RISE,70,32,2,4,NEW,'70 x 32 camera support plate')
    # A 0.8:1 flank gives a self-supporting printed dovetail undercut.
    pts=lambda half:[(-half,-19),(half,-19),(half,19),(-half,19)]
    loft(c,4.3+RISE,pts(20),9.3+RISE,pts(16),JOIN,'40 to 32 mm male dovetail')
    circle(c,4.3+RISE,0,0,6.8,9.0,CUT,'1/4 inch metal screw clearance 6.8 mm')
    circle(c,4.3+RISE,0,0,14,3.5,CUT,'14 x 3.5 mm screw head recess')
    # Small supports under the frontmost shell keep the two screw pillars joined.
    if frame:
        holexs=[-27.83906,27.83905]
        zholes=[13.3+RISE+4.72296,13.3+RISE+44.99704]
        for x in holexs:
            rect(c,13.3+RISE,x-4,-14.00043,x+4,-11.60043,49.0,JOIN,'Front four screw support column')
        pl=offset(c,14.00043,c.xZConstructionPlane)
        s=c.sketches.add(pl);s.name='Measured front screw holes 55.67811 x 40.27408'
        # XZ sketch orientation is queried, so no axis sign is assumed.
        for x in holexs:
            for z in zholes:
                center=s.modelToSketchSpace(p(x,-14.00043,z))
                s.sketchCurves.sketchCircles.addByCenterRadius(center,0.14)
        s.isVisible=False
        profiles=adsk.core.ObjectCollection.create()
        for profile in s.profiles:profiles.add(profile)
        origin=s.sketchToModelSpace(adsk.core.Point3D.create(0,0,0))
        normal_point=s.sketchToModelSpace(adsk.core.Point3D.create(0,0,1))
        ext(c,profiles,2.4 if normal_point.y>origin.y else -2.4,CUT,'Four 2.8 mm auxiliary screw clearances')
    rotate_body(c)
    body(c).name=name
    return o

def base_copy(root,name,source_body,translation,turn=False):
    o=component(root,name);c=o.component
    m=adsk.fusion.TemporaryBRepManager.get();b=m.copy(source_body)
    t=adsk.core.Matrix3D.create()
    if turn:t.setToRotation(math.pi/2,adsk.core.Vector3D.create(0,0,1),p(0,0,0))
    t.translation=adsk.core.Vector3D.create(*[v/10 for v in translation])
    if not m.transform(b,t):raise RuntimeError('Reference body transform failed')
    f=c.features.baseFeatures.add();f.startEdit()
    result=c.bRepBodies.add(b,f);f.finishEdit();result.name=name
    return o

def intersection(a,b):
    mgr=adsk.fusion.TemporaryBRepManager.get();t=mgr.copy(a)
    try:
        ok=mgr.booleanOperation(t,mgr.copy(b),adsk.fusion.BooleanTypes.IntersectionBooleanType)
    except RuntimeError as error:
        if 'ASM_EDGECOIN_PROBLEM' not in str(error):raise
        # Seating faces intentionally touch. Clear them by only 5 microns for
        # this diagnostic retry; the real assembly geometry is not moved.
        t=mgr.copy(a);other=mgr.copy(b)
        shift=adsk.core.Matrix3D.create();shift.translation=adsk.core.Vector3D.create(-0.0005,0.0005,0.0005)
        mgr.transform(other,shift)
        ok=mgr.booleanOperation(t,other,adsk.fusion.BooleanTypes.IntersectionBooleanType)
    if not ok: raise RuntimeError('Intersection check failed')
    if t.faces.count and t.volume*1000>0.01:
        bb=t.boundingBox
        debug={'a':a.name,'b':b.name,'volume_mm3':t.volume*1000,'bbox_mm':[[v*10 for v in [bb.minPoint.x,bb.minPoint.y,bb.minPoint.z]],[v*10 for v in [bb.maxPoint.x,bb.maxPoint.y,bb.maxPoint.z]]]}
        with open(os.path.join(OUT,'intersection_diagnostics.jsonl'),'a',encoding='utf8') as f:f.write(json.dumps(debug)+'\n')
    return t.volume*1000 if t.faces.count else 0.0

def export(design,obj,stem,stl=True):
    design.activateRootComponent()
    previous=[(o,o.isLightBulbOn) for o in design.rootComponent.occurrences]
    for o,_ in previous:o.isLightBulbOn=o==obj
    em=design.exportManager;res={}
    component_obj=obj.component
    for suffix,opts in [('f3d',em.createFusionArchiveExportOptions(os.path.join(OUT,stem+'.f3d'),component_obj)),('step',em.createSTEPExportOptions(os.path.join(OUT,stem+'.step'),component_obj))]:
        path=os.path.join(OUT,stem+'.'+suffix)
        if not em.execute(opts) or not os.path.getsize(path):raise RuntimeError('Failed export '+path)
        res[suffix]=path
    if stl:
        path=os.path.join(OUT,stem+'.stl')
        opts=em.createSTLExportOptions(body(component_obj),path)
        opts.meshRefinement=adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opts.sendToPrintUtility=False
        if not em.execute(opts) or not os.path.getsize(path):raise RuntimeError('Failed STL '+path)
        res['stl']=path
    for o,visible in previous:o.isLightBulbOn=visible
    return res

def fit_coupon(root,gap):
    o=component(root,'Fit_coupon_gap_'+str(gap));c=o.component
    rect(c,0,-25,-9,25,9,2,NEW,'Coupon base')
    for sign in [-1,1]:
        def q(edge):
            xs=sorted([sign*edge,sign*25]);return [(xs[0],-9),(xs[1],-9),(xs[1],9),(xs[0],9)]
        loft(c,2,q(20.24+gap),7.2,q(16.08+gap),JOIN,'Coupon dovetail rail')
    return o

def build(app,cmd):
    os.makedirs(OUT,exist_ok=True)
    # Capture source handles before creating a new document.
    camera_docs=[doc for doc in app.documents if doc.name=='MaixCAM2_3.0']
    if len(camera_docs)!=1:raise RuntimeError('Need the original MaixCAM2_3.0 document open')
    cam_design=adsk.fusion.Design.cast(camera_docs[0].products.itemByProductType('DesignProductType'))
    shell=[o for o in cam_design.rootComponent.allOccurrences if '+外壳:1+' in o.fullPathName and o.bRepBodies.count]
    if len(shell)!=3:raise RuntimeError('Expected the three original camera casing parts')
    connectors=[]
    for source_doc in app.documents:
        source_design=adsk.fusion.Design.cast(source_doc.products.itemByProductType('DesignProductType'))
        if source_design and source_design.rootComponent.allOccurrences.count==1:
            connectors += [o for o in source_design.rootComponent.allOccurrences if o.bRepBodies.count==1 and o.bRepBodies.item(0).faces.count==55]
    if len(connectors)!=1:raise RuntimeError('Activate the imported 云台连接件 document first')
    connector_body=connectors[0].bRepBodies.item(0)
    doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name='MaixCAM2_quick_release_v2'
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    report={'version':'v2','source_camera':'MaixCAM2_3.0.step','source_gimbal':'云台连接件.step','dimensions_mm':{'receiver':[44,56,9.2],'plate':[38,70,9],'plate_top':13.3,'rail_gap_each_side':0.25,'friction_interference_each_side':0.10,'mount_slot_width':2.4,'mount_slot_length_between_end_centres':65,'arm_thickness':2,'heat_pin_diameter':2.1,'heat_pin_length':4.5,'pins_y':[-16,16],'centre_screw_clearance':6.8,'head_recess_diameter':14,'head_recess_depth':3.5,'camera_screw_bearing_thickness':5.5,'four_screw_pitch':[55.67811,40.27408],'four_screw_clearance':2.8,'front_frame_added_thickness':2.4},'exports':{},'checks':{}}
    mount=receiver(root,'Receiver_heat_stake_v2',True)
    bolt=receiver(root,'Receiver_M2_bolt_v2',False)
    simple=plate(root,'Camera_plate_1_4_v2',False)
    frame=plate(root,'Camera_plate_1_4_four_screw_v2',True)
    bracket=base_copy(root,'Reference_original_gimbal_connector',connector_body,[1.68997393867119,-130.07520270418,-48.461489632961])
    refs=[]
    for i,o in enumerate(shell):
        for b in o.bRepBodies:
            refs.append(base_copy(root,'Reference_camera_casing_'+str(i),b,[4.25,0,38.16+RISE],True))
    finish(app,report)

def finish(app,report=None):
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    def get(name):
        found=[o for o in root.occurrences if o.component.name==name]
        if len(found)!=1:raise RuntimeError('Expected unique component '+name)
        return found[0]
    mount=get('Receiver_heat_stake_v2');bolt=get('Receiver_M2_bolt_v2')
    simple=get('Camera_plate_1_4_v2');frame=get('Camera_plate_1_4_four_screw_v2')
    bracket=get('Reference_original_gimbal_connector')
    refs=[o for o in root.occurrences if o.component.name.startswith('Reference_camera_casing_')]
    if report is None:
        report={'version':'v2','source_camera':'MaixCAM2_3.0.step','source_gimbal':'云台连接件.step','dimensions_mm':{'receiver':[44,56,9.2],'plate':[38,70,9],'plate_top':13.3,'rail_gap_each_side':0.25,'friction_interference_each_side':0.10,'mount_slot_width':2.4,'mount_slot_length_between_end_centres':65,'arm_thickness':2,'heat_pin_diameter':2.1,'heat_pin_length':4.5,'pins_y':[-16,16],'centre_screw_clearance':6.8,'head_recess_diameter':14,'head_recess_depth':3.5,'camera_screw_bearing_thickness':5.5,'four_screw_pitch':[55.67811,40.27408],'four_screw_clearance':2.8,'front_frame_added_thickness':2.4},'exports':{},'checks':{}}
    report['orientation']={'camera_long_edge_axis':'Y','gimbal_arm_long_axis':'Y','insert_direction':'-X','withdraw_direction':'+X','underside_mount_pins_remain_along_original_Y_slot':True}
    report['dimensions_mm']['receiver']=[44,56,9.2+RISE]
    report['dimensions_mm']['plate_top']=13.3+RISE
    report['checks']['coincident_face_retry_offset_xyz_mm']=[-0.005,0.005,0.005]
    appearance(app,mount.component,[50,90,115]);appearance(app,frame.component,[218,133,53])
    for o in [mount,bolt,simple,frame]:
        b=body(o.component);bb=b.boundingBox
        report['checks'][o.component.name]={'one_solid':True,'volume_mm3':b.volume*1000,'bbox_mm':[[v*10 for v in [bb.minPoint.x,bb.minPoint.y,bb.minPoint.z]],[v*10 for v in [bb.maxPoint.x,bb.maxPoint.y,bb.maxPoint.z]]],'features':o.component.features.extrudeFeatures.count+o.component.features.loftFeatures.count}
    rb=body(mount.component);pb=body(frame.component);gb=body(bracket.component)
    report['checks']['receiver_to_original_arm_intersection_mm3']=intersection(rb,gb)
    report['checks']['plate_to_original_arm_intersection_mm3']=intersection(pb,gb)
    report['checks']['plate_to_camera_casing_intersections_mm3']=[intersection(pb,body(o.component)) for o in refs]
    report['checks']['receiver_to_camera_casing_intersections_mm3']=[intersection(rb,body(o.component)) for o in refs]
    report['checks']['camera_casing_to_original_arm_intersections_mm3']=[intersection(body(o.component),gb) for o in refs]
    report['checks']['receiver_to_plate_designed_friction_interference_mm3']=intersection(rb,pb)
    with open(os.path.join(OUT,'geometry_checks.json'),'w',encoding='utf8') as f:json.dump(report, f, indent=2)
    for key in ['receiver_to_original_arm_intersection_mm3','plate_to_original_arm_intersection_mm3']:
        if report['checks'][key]>0.01:raise RuntimeError('Unintended arm collision '+key+str(report['checks'][key]))
    if max(report['checks']['plate_to_camera_casing_intersections_mm3'])>0.01:raise RuntimeError('Camera frame collision')
    for o,stem in [(mount,'receiver_heat_stake'),(bolt,'receiver_M2_bolt'),(simple,'camera_plate'),(frame,'camera_plate_four_screw')]:
        report['exports'][stem]=export(design,o,stem)
    coupons=[]
    for gap in [.15,.25,.35]:
        found=[o for o in root.occurrences if o.component.name=='Fit_coupon_gap_'+str(gap)]
        o=found[0] if found else fit_coupon(root,gap)
        coupons.append(o);report['exports']['coupon_'+str(gap)]=export(design,o,'fit_coupon_gap_'+str(gap))
        o.isLightBulbOn=False
    found=[o for o in root.occurrences if o.component.name=='Fit_coupon_male']
    if found:male=found[0]
    else:
        male=component(root,'Fit_coupon_male');c=male.component
        loft(c,2.3,[(-20,-9),(20,-9),(20,9),(-20,9)],7.3,[(-16,-9),(16,-9),(16,9),(-16,9)],NEW,'Coupon male dovetail')
    report['exports']['coupon_male']=export(design,male,'fit_coupon_male');male.isLightBulbOn=False
    for o in root.occurrences:
        o.component.isConstructionFolderLightBulbOn=False
        o.component.isSketchFolderLightBulbOn=False
        o.component.opacity=1.0
        o.isLightBulbOn=o in [mount,frame,bracket]+refs
        for b in o.component.bRepBodies:b.opacity=1.0
    design.activateRootComponent()
    app.activeViewport.visualStyle=adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    cam=app.activeViewport.camera
    cam.eye=p(160,-120,110);cam.target=p(0,-10,27);cam.upVector=adsk.core.Vector3D.create(0,0,1);cam.isFitView=True
    app.activeViewport.camera=cam;app.activeViewport.fit();app.activeViewport.refresh()
    archive=os.path.join(OUT,'MaixCAM2_quick_release_assembly.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(archive,root)):raise RuntimeError('Assembly export failed')
    report['exports']['assembly_f3d']=archive
    path=os.path.join(OUT,'MaixCAM2_quick_release_assembly.step')
    if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(path,root)):raise RuntimeError('Assembly STEP export failed')
    report['exports']['assembly_step']=path
    image_path=os.path.join(OUT,'assembly_preview.png')
    report['preview_saved']=app.activeViewport.saveAsImageFile(image_path,1600,1100)
    report['physical_validation']='Not printed or tested. Rail friction and fastener engagement require physical checks.'
    with open(os.path.join(OUT,'build_report.json'),'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)

def presentation(app):
    """Refresh saved assembly appearance without rebuilding printable parts."""
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    for o in root.occurrences:
        o.component.opacity=1.0
        for b in o.component.bRepBodies:b.opacity=1.0
    design.activateRootComponent()
    app.activeViewport.visualStyle=adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    app.activeViewport.refresh()
    report=json.load(open(os.path.join(OUT,'build_report.json'),encoding='utf8'))
    report['visible_opacity']={o.component.name:o.visibleOpacity for o in root.occurrences if o.isVisible}
    for path,options in [
        (report['exports']['assembly_f3d'],design.exportManager.createFusionArchiveExportOptions(report['exports']['assembly_f3d'],root)),
        (report['exports']['assembly_step'],design.exportManager.createSTEPExportOptions(report['exports']['assembly_step'],root))]:
        if not design.exportManager.execute(options):raise RuntimeError('Assembly presentation export failed '+path)
    report['preview_saved']=app.activeViewport.saveAsImageFile(os.path.join(OUT,'assembly_preview.png'),1600,1100)
    with open(os.path.join(OUT,'build_report.json'),'w',encoding='utf8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
