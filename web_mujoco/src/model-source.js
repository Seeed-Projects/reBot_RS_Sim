export function parseIncludesAndMeshes(xmlText) {
  const files = new Set();
  const includeRe = /<include\s+file="([^"]+)"/gi;
  const meshRe = /\sfile="([^"]+\.(?:STL|stl|obj|msh))"/g;
  let match;
  while ((match = includeRe.exec(xmlText))) files.add(match[1]);
  while ((match = meshRe.exec(xmlText))) {
    const relative = match[1].replaceAll('\\', '/').replace(/^\.\//, '');
    files.add(relative.startsWith('meshes/') ? relative : `meshes/${relative}`);
  }
  return [...files];
}

export function extractMaterialProps(files, sceneXmlName) {
  const props = [];

  function parseSegment(text) {
    const re = /<material\s+([^>]*)\/?>/gi;
    let m;
    while ((m = re.exec(text))) {
      const attrs = m[1];
      const name = attrs.match(/name="([^"]+)"/)?.[1] ?? '';
      const met = parseFloat(attrs.match(/metallic="([^"]+)"/)?.[1] ?? '');
      const rou = parseFloat(attrs.match(/roughness="([^"]+)"/)?.[1] ?? '');
      props.push({
        name,
        metallic: Number.isFinite(met) ? met : null,
        roughness: Number.isFinite(rou) ? rou : null
      });
    }
  }

  function parseWithIncludes(xmlText) {
    const includeRe = /<include\s+file="([^"]+)"\s*\/?>/gi;
    let last = 0;
    let m;
    while ((m = includeRe.exec(xmlText))) {
      parseSegment(xmlText.slice(last, m.index));
      const file = m[1];
      if (files[file]) {
        const txt = new TextDecoder().decode(files[file]);
        parseWithIncludes(txt.replace(/<\/?mujoco[^>]*>/g, ''));
      }
      last = m.index + m[0].length;
    }
    parseSegment(xmlText.slice(last));
  }

  if (files[sceneXmlName]) {
    parseWithIncludes(new TextDecoder().decode(files[sceneXmlName]));
  }
  return props;
}
