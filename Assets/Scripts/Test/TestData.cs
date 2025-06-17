using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class TestData
{
    public string description;
    public List<float> distances = new List<float>();
    public List<float> errors = new List<float>();
    public List<int> iterations = new List<int>();
    public List<float> constraintErrors = new List<float>();
    public List<float> initialSpeeds = new List<float>();
    public List<float> zSpin = new List<float>();
    public List<float> ySpin = new List<float>();
    public List<double> computationTime = new List<double>();
    
    public void SaveToJson(String filename)
    {
        var path = "C:\\Users\\ricca\\Documents\\GitHub\\Controllable-Ball-Physics-Simulation\\Assets\\Tests\\" + filename + ".json";
        string json = JsonUtility.ToJson(this, true);
        File.WriteAllText(path, json);
        Debug.Log("Test data saved at: " + path);
    }
}