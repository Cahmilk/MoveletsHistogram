package br.ufsc.trajectoryclassification.model.vo.description;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public class JsonToJava {

    public static void main(String[] args) throws IOException {
        try(Reader reader = new InputStreamReader(new FileInputStream("D:/Users/andres/git_projects/datasets/00_my1/data-r2/description.json"), "UTF-8")){
            Gson gson = new GsonBuilder().create();
            Description p = gson.fromJson(reader, Description.class);
            System.out.println(p);
        }
    }
}