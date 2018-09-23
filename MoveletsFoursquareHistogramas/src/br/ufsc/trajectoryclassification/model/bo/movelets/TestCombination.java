package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.Arrays;

import org.apache.commons.math3.util.Combinations;

public class TestCombination {

	
	public static void main(String[] args) {
		
		Combinations c = new Combinations(5,2);
		for (int[] is : c) {
			System.out.println(Arrays.toString(is) );
		}
		
	}
}
