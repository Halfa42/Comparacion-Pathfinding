using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Xml.Linq;

// Minimal XLSX export using the runtime libraries already available in Unity.
public static class BenchmarkWorkbook
{
    public const string Header = "Ejecución,Éxito,Nodos Expandidos (Alto Nivel),Tiempo (ms),Suma de Costos,Makespan,RAM Pico (MB)";
    private static readonly XNamespace Sheet = "http://schemas.openxmlformats.org/spreadsheetml/2006/main";
    private static readonly XNamespace Office = "http://schemas.openxmlformats.org/officeDocument/2006/relationships";
    private static readonly XNamespace Package = "http://schemas.openxmlformats.org/package/2006/relationships";
    private static readonly XNamespace Content = "http://schemas.openxmlformats.org/package/2006/content-types";

    public static void Save(string path, IReadOnlyDictionary<string, List<string[]>> results)
    {
        if (results.Count == 0) return;
        using var buffer = new MemoryStream();
        using (var archive = new ZipArchive(buffer, ZipArchiveMode.Create, true))
        {
            var sheets = new XElement(Sheet + "sheets");
            var relationships = new XElement(Package + "Relationships");
            var types = new XElement(Content + "Types",
                new XElement(Content + "Default", new XAttribute("Extension", "rels"),
                    new XAttribute("ContentType", "application/vnd.openxmlformats-package.relationships+xml")),
                new XElement(Content + "Default", new XAttribute("Extension", "xml"), new XAttribute("ContentType", "application/xml")),
                new XElement(Content + "Override", new XAttribute("PartName", "/xl/workbook.xml"),
                    new XAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml")));
            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            int index = 0;
            foreach (var algorithm in results)
            {
                index++;
                string id = "rId" + index, target = "worksheets/sheet" + index + ".xml";
                sheets.Add(new XElement(Sheet + "sheet", new XAttribute("name", SheetName(algorithm.Key, names)),
                    new XAttribute("sheetId", index), new XAttribute(Office + "id", id)));
                relationships.Add(new XElement(Package + "Relationship", new XAttribute("Id", id),
                    new XAttribute("Type", Office.NamespaceName + "/worksheet"), new XAttribute("Target", target)));
                types.Add(new XElement(Content + "Override", new XAttribute("PartName", "/xl/" + target),
                    new XAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml")));

                string[] headers = Header.Split(',');
                var columns = new XElement(Sheet + "cols", headers.Select((label, column) =>
                    new XElement(Sheet + "col", new XAttribute("min", column + 1), new XAttribute("max", column + 1),
                        new XAttribute("width", Math.Max(14, label.Length + 2)), new XAttribute("customWidth", 1))));
                var data = new XElement(Sheet + "sheetData", Row(headers, 1, true));
                for (int row = 0; row < algorithm.Value.Count; row++) data.Add(Row(algorithm.Value[row], row + 2, false));
                Write(archive, "xl/" + target, new XElement(Sheet + "worksheet", columns, data,
                    new XElement(Sheet + "autoFilter", new XAttribute("ref", "A1:G" + (algorithm.Value.Count + 1)))));
            }
            Write(archive, "[Content_Types].xml", types);
            Write(archive, "_rels/.rels", new XElement(Package + "Relationships",
                new XElement(Package + "Relationship", new XAttribute("Id", "rId1"),
                    new XAttribute("Type", Office.NamespaceName + "/officeDocument"), new XAttribute("Target", "xl/workbook.xml"))));
            Write(archive, "xl/workbook.xml", new XElement(Sheet + "workbook", new XAttribute(XNamespace.Xmlns + "r", Office), sheets));
            Write(archive, "xl/_rels/workbook.xml.rels", relationships);
        }
        // Build in memory, then write. No temporary files are needed.
        File.WriteAllBytes(path, buffer.ToArray());
    }

    private static XElement Row(string[] values, int row, bool header)
    {
        return new XElement(Sheet + "row", new XAttribute("r", row), values.Select((value, column) =>
            new XElement(Sheet + "c", new XAttribute("r", ((char)('A' + column)).ToString() + row),
                new XAttribute("t", header ? "inlineStr" : column == 1 ? "b" : "n"),
                header ? new XElement(Sheet + "is", new XElement(Sheet + "t", value)) :
                    new XElement(Sheet + "v", column == 1 ? (value == "True" ? "1" : "0") : value))));
    }

    private static string SheetName(string label, HashSet<string> used)
    {
        string name = new string(label.Select(c => "[]:*?/\\".Contains(c) || char.IsControl(c) ? '_' : c).ToArray()).Trim('\'');
        if (string.IsNullOrWhiteSpace(name)) name = "Algoritmo";
        name = name.Substring(0, Math.Min(31, name.Length));
        string candidate = name;
        for (int suffix = 2; !used.Add(candidate); suffix++)
        {
            string ending = "_" + suffix;
            candidate = name.Substring(0, Math.Min(name.Length, 31 - ending.Length)) + ending;
        }
        return candidate;
    }

    private static void Write(ZipArchive archive, string path, XElement root)
    {
        using var stream = archive.CreateEntry(path).Open();
        new XDocument(root).Save(stream);
    }
}
