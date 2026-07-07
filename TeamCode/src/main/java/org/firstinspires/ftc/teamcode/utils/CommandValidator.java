package org.firstinspires.ftc.teamcode.utils;

import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.expr.MethodCallExpr;
import com.github.javaparser.ast.stmt.ExpressionStmt;
import com.github.javaparser.ast.visitor.VoidVisitorAdapter;
import com.github.javaparser.resolution.types.ResolvedType;
import com.github.javaparser.symbolsolver.JavaSymbolSolver;
import com.github.javaparser.symbolsolver.resolution.typesolvers.CombinedTypeSolver;
import com.github.javaparser.symbolsolver.resolution.typesolvers.JavaParserTypeSolver;
import com.github.javaparser.symbolsolver.resolution.typesolvers.ReflectionTypeSolver;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class CommandValidator {

    public static void scanProject() {
        File teamCodeDir = new File("TeamCode/src/main/java");

        if (!teamCodeDir.exists() || !teamCodeDir.isDirectory()) {
            System.out.println("Warning: Could not find TeamCode directory.");
            return;
        }

        // Set up the Type Solver so JavaParser can look up superclasses
        CombinedTypeSolver typeSolver = new CombinedTypeSolver();
        typeSolver.add(new ReflectionTypeSolver()); // For standard Java types
        typeSolver.add(new JavaParserTypeSolver(teamCodeDir)); // For your TeamCode types

        JavaSymbolSolver symbolSolver = new JavaSymbolSolver(typeSolver);
        StaticJavaParser.getConfiguration().setSymbolResolver(symbolSolver);

        List<String> violations = new ArrayList<>();
        scanDirectory(teamCodeDir, violations);

        if (!violations.isEmpty()) {
            StringBuilder errorMsg = new StringBuilder();
            errorMsg.append("\n=======================================================\n");
            errorMsg.append("STATIC SCANNER ERROR: IGNORED MUST-USE RETURN VALUE!\n");
            for (String v : violations) {
                errorMsg.append(v).append("\n");
            }
            errorMsg.append("=======================================================\n");
            throw new IllegalStateException(errorMsg.toString());
        }
    }

    private static void scanDirectory(File directory, List<String> violations) {
        File[] files = directory.listFiles();
        if (files == null) return;

        for (File file : files) {
            if (file.isDirectory()) {
                scanDirectory(file, violations);
            } else if (file.getName().endsWith(".java")) {
                try {
                    CompilationUnit cu = StaticJavaParser.parse(file);

                    cu.accept(new VoidVisitorAdapter<Void>() {
                        @Override
                        public void visit(ExpressionStmt n, Void arg) {
                            super.visit(n, arg);

                            if (n.getExpression() instanceof MethodCallExpr) {
                                MethodCallExpr methodCall = (MethodCallExpr) n.getExpression();

                                try {
                                    // Ask the symbol solver what type this method invocation actually returns
                                    ResolvedType returnType = methodCall.calculateResolvedType();

                                    // Check if the type or its ancestry describes your Command class
                                    if (isCommandSubclass(returnType)) {
                                        String fileLocation = file.getName();
                                        int line = n.getBegin().map(pos -> pos.line).orElse(-1);

                                        violations.add("-> Ignored " + returnType.describe() + " return in "
                                                + fileLocation + " at line " + line + ": `" + n.toString().trim() + "`");
                                    }
                                } catch (Exception e) {
                                    // Symbol solving can fail if code has compile syntax errors; skip safely
                                }
                            }
                        }
                    }, null);

                } catch (Exception e) {
                    // Skip unparseable files
                }
            }
        }
    }

    /**
     * Recursively travels up the type ancestry to see if it derives from your Command base class.
     */
    private static boolean isCommandSubclass(ResolvedType type) {
        if (type == null) return false;

        String typeName = type.describe();

        // Match against your exact library package structure
        if (typeName.equals("com.mylibrary.Command") || typeName.contains(".Command")) {
            return true;
        }

        // Recursively check superclasses/ancestors for polymorphism mapping
        if (type.isReferenceType()) {
            for (ResolvedType ancestor : type.asReferenceType().getAllAncestors()) {
                if (ancestor.describe().equals("com.mylibrary.Command") || ancestor.describe().contains(".Command")) {
                    return true;
                }
            }
        }

        return false;
    }
}