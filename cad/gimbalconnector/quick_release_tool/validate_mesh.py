"""Inspect exported binary STL topology and real millimetre envelopes."""
import struct, json, pathlib, collections, math

OUT=pathlib.Path(__file__).resolve().parent.parent/'output_v2'

def check(path):
    data=path.read_bytes(); n=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*n:raise ValueError('Invalid binary STL size '+str(path))
    vertices=[];faces=[];index={};degenerate=0;volume=0
    for k in range(n):
        row=struct.unpack_from('<12fH',data,84+50*k)
        pts=[tuple(row[j:j+3]) for j in (3,6,9)]
        face=[]
        for v in pts:
            key=tuple(round(x,5) for x in v)
            if key not in index:index[key]=len(vertices);vertices.append(v)
            face.append(index[key])
        if len(set(face))<3:degenerate+=1
        faces.append(face)
        a,b,c=pts
        volume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
    edges=collections.defaultdict(list);orientation=collections.Counter()
    for i,face in enumerate(faces):
        for a,b in zip(face,face[1:]+face[:1]):
            edge=tuple(sorted((a,b)));edges[edge].append(i);orientation[edge]+=1 if a<b else -1
    parents=list(range(n))
    def find(i):
        while parents[i]!=i:parents[i]=parents[parents[i]];i=parents[i]
        return i
    for f in edges.values():
        for b in f[1:]:parents[find(b)]=find(f[0])
    lo=[min(v[i] for v in vertices) for i in range(3)];hi=[max(v[i] for v in vertices) for i in range(3)]
    return {'triangles':n,'vertices':len(vertices),'boundary_edges':sum(len(f)==1 for f in edges.values()),'nonmanifold_edges':sum(len(f)>2 for f in edges.values()),'inconsistent_edges':sum(v!=0 for v in orientation.values()),'connected_meshes':len({find(i) for i in range(n)}),'degenerate_triangles':degenerate,'bbox_mm':[lo,hi],'dimensions_mm':[b-a for a,b in zip(lo,hi)],'volume_mm3':volume}

def main():
    report={p.name:check(p) for p in sorted(OUT.glob('*.stl'))}
    if len(report)!=8:raise ValueError('Expected 8 printable STL exports, found '+str(len(report)))
    build=json.loads((OUT/'build_report.json').read_text(encoding='utf8'))
    names={'receiver_heat_stake.stl':'Receiver_heat_stake_v2','receiver_M2_bolt.stl':'Receiver_M2_bolt_v2','camera_plate.stl':'Camera_plate_1_4_v2','camera_plate_four_screw.stl':'Camera_plate_1_4_four_screw_v2'}
    for name,comp in names.items():
        mesh=report[name];cad=build['checks'][comp]
        dimensions=[b-a for a,b in zip(*cad['bbox_mm'])]
        mesh['cad_dimension_error_mm']=max(abs(a-b) for a,b in zip(dimensions,mesh['dimensions_mm']))
        mesh['relative_volume_error']=abs(mesh['volume_mm3']/cad['volume_mm3']-1)
    for name,v in report.items():
        v['passed']=v['boundary_edges']==0 and v['nonmanifold_edges']==0 and v['inconsistent_edges']==0 and v['connected_meshes']==1 and v['degenerate_triangles']==0 and v['volume_mm3']>0 and v.get('cad_dimension_error_mm',0)<.02 and v.get('relative_volume_error',0)<.005
    (OUT/'mesh_validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))
    if not all(v['passed'] for v in report.values()):raise ValueError('STL validation failed')

if __name__=='__main__':main()
