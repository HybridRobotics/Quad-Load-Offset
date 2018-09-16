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
legend({'Geometric control design $\mathcal{M}_Z$','Geometric control design $\mathcal{M}_O$'},'Interpreter','latex')