using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using System.Linq;

public class SaveSystem
{
    public void SaveToJson(String fileName,String description, List<Vector3> startPositions, List<float> distances, List<float> errors, List<Vector3> outliersStartPositions, List<float> outliersErrors, float averageTimeToCompute, List<int> iterations)
    {
        var path = "C:\\Users\\ricca\\Documents\\GitHub\\My project\\Assets\\Tests\\" + fileName + ".json";
        TestData data = new TestData {description = description, startPositions = startPositions, distances = distances, errors = errors, outliersStartPositions = outliersStartPositions, outliersErrors = outliersErrors, averageTimeToCompute = averageTimeToCompute, iterations = iterations };
        data.ComputeAllFields();
        string json = JsonUtility.ToJson(data, true);
        File.WriteAllText(path, json);
        Debug.Log("Data saved at: " + path);
    }
}

[System.Serializable]
public class TestData
{
    public String description;
    public List<Vector3> startPositions = new List<Vector3>();
    public List<float> distances = new List<float>();
    public List<float> errors = new List<float>();
    public List<Vector3> outliersStartPositions = new List<Vector3>();
    public List<float> outliersErrors = new List<float>();
    public float average;
    public float variance;
    public float standardDeviation;
    public float median;
    public float min;
    public float max;
    public float averageTimeToCompute;
    public List<int> iterations = new List<int>();

    public void ComputeAllFields()
    {
        if (errors == null || errors.Count == 0)
        {
            average = variance = standardDeviation = median = min = max = 0;
            return;
        }

        // Min e Max
        min = errors.Min();
        max = errors.Max();

        // Media
        average = errors.Average();

        // Varianza
        float sumOfSquares = errors.Sum(x => (x - average) * (x - average));
        variance = sumOfSquares / (errors.Count-1);

        // Deviazione standard
        standardDeviation = (float)Math.Sqrt(variance);

        // Mediana
        List<float> sortedErrors = errors.OrderBy(x => x).ToList();
        int count = sortedErrors.Count;
        if (count % 2 == 0)
        {
            // Se la lunghezza è pari, facciamo la media dei due valori centrali
            median = (sortedErrors[count / 2] + sortedErrors[count / 2 + 1]) / 2f;
        }
        else
        {
            median = sortedErrors[(count + 1) / 2];
        }
    }
}

