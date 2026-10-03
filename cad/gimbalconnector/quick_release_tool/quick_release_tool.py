import adsk.core, adsk.fusion
import os, json, traceback

HERE = os.path.dirname(os.path.abspath(__file__))

def xyz(p):
    return [round(p.x*10,5), round(p.y*10,5), round(p.z*10,5)]

def box(b):
    return [xyz(b.minPoint), xyz(b.maxPoint)]

def body_info(b, detailed=True):
    out = {'name':b.name,'bbox_mm':box(b.boundingBox),'solid':b.isSolid,'faces':b.faces.count}
    if detailed:
        out['cylinders']=[]
        out['planes']=[]
        for f in b.faces:
            g=f.geometry
            if g.objectType==adsk.core.Cylinder.classType() and 0.08<g.radius<0.8:
                out['cylinders'].append({'origin':xyz(g.origin),'axis':[g.axis.x,g.axis.y,g.axis.z],'r':g.radius*10,'bbox':box(f.boundingBox),'area':f.area*100})
            elif g.objectType==adsk.core.Plane.classType() and f.area>0.5:
                out['planes'].append({'origin':xyz(g.origin),'normal':[g.normal.x,g.normal.y,g.normal.z],'bbox':box(f.boundingBox),'area':f.area*100})
    return out

def inspect(app):
    report={'active':app.activeDocument.name,'documents':[]}
    for doc in app.documents:
        d=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        if not d: continue
        root=d.rootComponent
        item={'name':doc.name,'root':root.name,'bodies':[body_info(b) for b in root.bRepBodies],'occurrences':[]}
        for o in root.allOccurrences:
            item['occurrences'].append({'name':o.fullPathName,'visible':o.isVisible,'transform':o.transform2.asArray(),'bbox_mm':box(o.boundingBox),'bodies':[body_info(b) for b in o.bRepBodies]})
        report['documents'].append(item)
    with open(os.path.join(HERE,'inspection.json'),'w',encoding='utf-8') as f: json.dump(report,f,ensure_ascii=False,indent=2)

def run(context):
    app=adsk.core.Application.get()
    try:
        cmd_path=os.path.join(HERE,'command.json')
        cmd=json.load(open(cmd_path,encoding='utf-8')) if os.path.exists(cmd_path) else {'mode':'inspect'}
        if cmd['mode']=='inspect': inspect(app)
        elif cmd['mode']=='probe_connector':
            opts=app.importManager.createSTEPImportOptions(os.path.join(HERE,'..','云台连接件.step'))
            doc=app.importManager.importToNewDocument(opts)
            design=adsk.fusion.Design.cast(app.activeProduct)
            out={'document':app.activeDocument.name,'bodies':[body_info(b) for b in design.rootComponent.bRepBodies],'occurrences':[]}
            for o in design.rootComponent.allOccurrences:
                out['occurrences'].append({'name':o.fullPathName,'transform':o.transform2.asArray(),'bodies':[body_info(b) for b in o.bRepBodies]})
            with open(os.path.join(HERE,'connector_geometry.json'),'w',encoding='utf-8') as f:json.dump(out,f,ensure_ascii=False,indent=2)
        else:
            import sys, importlib
            if HERE not in sys.path: sys.path.insert(0,HERE)
            if cmd['mode'] in ['build_reinforced','build_usb','finish_optimized','preview_optimized','repair_optimized','clean_optimized','trim_optimized']:
                import build_optimized
                importlib.reload(build_optimized)
                build_optimized.configure(cmd.get('variant','usb' if cmd['mode']=='build_usb' else 'reinforced'))
                if cmd['mode']=='trim_optimized':build_optimized.trim(app)
                elif cmd['mode']=='clean_optimized':build_optimized.clean(app)
                elif cmd['mode']=='repair_optimized':build_optimized.repair(app)
                elif cmd['mode']=='finish_optimized':build_optimized.finish(app)
                elif cmd['mode']=='preview_optimized':build_optimized.presentation(app)
                else:build_optimized.build(app,cmd)
                app.userInterface.messageBox('Optimized adapter completed. Original V2 preserved.','MaixCAM2 adapter')
                return
            import build_adapter
            importlib.reload(build_adapter)
            if cmd['mode']=='preview':build_adapter.presentation(app)
            elif cmd['mode']=='finish':build_adapter.finish(app)
            else:build_adapter.build(app,cmd)
        app.userInterface.messageBox('Quick release tool completed. Local report saved.','MaixCAM2 adapter')
    except:
        error=traceback.format_exc()
        with open(os.path.join(HERE,'error.txt'),'w',encoding='utf-8') as f:f.write(error)
        app.userInterface.messageBox(error,'MaixCAM2 adapter error')
