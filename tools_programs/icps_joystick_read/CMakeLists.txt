cmake_minimum_required(VERSION 3.0)
project(test)  # Change "YourProjectName" to your project's name

file( # liste des fichiers à compiler
	GLOB_RECURSE # recherche récursive
	source_files # fichiers listés dans une variable source_files
	src/*.cpp # forme des fichiers à rechercher
)

add_executable( # création de l’exécutable binaire du projet
	${PROJECT_NAME} # contient le nom du binaire
	${source_files} # liste des fichiers compilés pour le linking
)

