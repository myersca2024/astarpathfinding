using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class PathfindingMap : MonoBehaviour
{
    public Tilemap map;
    public Dictionary<Vector3Int, Vector3> walkableTiles;

    void Start()
    {
        walkableTiles = new Dictionary<Vector3Int, Vector3>();
        for (int xx = map.cellBounds.xMin; xx < map.cellBounds.xMax; xx++)
        {
            for (int yy = map.cellBounds.yMin; yy < map.cellBounds.yMax; yy++)
            {
                Vector3Int localPos = new Vector3Int(xx, yy, (int)map.transform.position.y);
                Vector3 pos = map.CellToWorld(localPos);
                if (map.HasTile(localPos))
                {
                    // Debug.Log("(" + xx + ", " + yy + ")");
                    walkableTiles.Add(localPos, pos);
                }
            }
        }
    }
}
