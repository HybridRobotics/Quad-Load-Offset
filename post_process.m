%% Plotting final error in function of orbit frequency
figure;
hold on
errorbar(freq,old,oldUncertainty,'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
errorbar(freq,new,newUncertainty,'-o','MarkerSize',10,...
    'MarkerEdgeColor','blue','MarkerFaceColor','blue')
set(gca,'YScale','log')
xlabel({'f (Hz)'},'Interpreter','latex')
ylabel({'$e_{x_L}$ (m)'},'Interpreter','latex')
grid on
legend({'Geometric controller $\mathcal{C}_Z$','Geometric controller $\mathcal{C}_O$'},'Interpreter','latex')