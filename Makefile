cc=g++
deps = geometry.h model.h tgaimage.h
prom = main.out
obj = main.o model.o tgaimage.o 
$(prom): $(obj)
	$(cc) -pg -o $(prom) $(obj)
check:
	./$(prom)|xdg-open output.tga
%.o: %.cpp $(deps)
	$(cc) -c $< -o $@
clean:
	rm -rf $(obj) $(prom)

